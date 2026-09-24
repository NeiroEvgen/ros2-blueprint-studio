"""
Профиль удалённой цели и подключение к ней.

Профиль хранится в проекте: <project>/.blueprint/remote.yaml.
Секретов в нём НЕТ: только хост, пользователь, путь к ключу. Пароль,
если нужен, передаётся в connect() в момент подключения и нигде не
сохраняется — поэтому remote.yaml можно спокойно коммитить.

Ключи хостов — TOFU (trust on first use), как у обычного ssh: при первом
подключении отпечаток запоминается в ~/.blueprint/known_hosts, при смене
ключа подключение ОТКАЗЫВАЕТСЯ. AutoAddPolicy здесь не годится: он молча
принял бы подменённую машину.
"""
import os
from dataclasses import dataclass, asdict, field

import yaml

PROFILE_REL = os.path.join(".blueprint", "remote.yaml")
DEFAULT_KNOWN_HOSTS = os.path.join(os.path.expanduser("~"), ".blueprint", "known_hosts")

_ALLOWED_KEYS = {"host", "port", "user", "key_path", "remote_workspace",
                 "container", "container_workspace", "ros_setup"}


@dataclass
class RemoteProfile:
    host: str = ""
    port: int = 22
    user: str = ""
    key_path: str = ""                  # пусто = ssh-agent / ~/.ssh
    remote_workspace: str = ""          # куда кладём исходники на хосте
    container: str = ""                 # пусто = собирать на хосте напрямую
    container_workspace: str = ""       # тот же каталог, как он виден в контейнере
    ros_setup: str = "/opt/ros/humble/setup.bash"

    @property
    def enabled(self):
        return bool(self.host and self.user and self.remote_workspace)

    @property
    def build_dir(self):
        """Каталог, где запускать colcon: в контейнере или на хосте."""
        if self.container:
            return self.container_workspace or self.remote_workspace
        return self.remote_workspace

    def display(self):
        return f"{self.user}@{self.host}" + (f":{self.port}" if self.port != 22 else "")

    def validate(self):
        errors = []
        if not self.host:
            errors.append("не указан хост")
        if not self.user:
            errors.append("не указан пользователь")
        if not self.remote_workspace:
            errors.append("не указан remote_workspace")
        elif not self.remote_workspace.startswith("/"):
            errors.append("remote_workspace должен быть абсолютным путём (/home/...)")
        if not (0 < int(self.port) < 65536):
            errors.append("некорректный порт")
        if self.container and self.container_workspace and not self.container_workspace.startswith("/"):
            errors.append("container_workspace должен быть абсолютным путём")
        return errors


class ProfileStore:
    def __init__(self, project_path):
        self.path = os.path.join(project_path, PROFILE_REL)

    def load(self):
        if not os.path.exists(self.path):
            return RemoteProfile()
        with open(self.path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        clean = {k: v for k, v in data.items() if k in _ALLOWED_KEYS}
        if "port" in clean:
            clean["port"] = int(clean["port"])
        return RemoteProfile(**clean)

    def save(self, profile):
        errors = profile.validate()
        if errors:
            raise ValueError("; ".join(errors))
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w", encoding="utf-8") as f:
            yaml.safe_dump(asdict(profile), f, sort_keys=False, allow_unicode=True)


class HostKeyChanged(Exception):
    pass


def _tofu_policy(known_hosts_path):
    import paramiko

    class TOFUPolicy(paramiko.MissingHostKeyPolicy):
        """Вызывается только для НЕизвестного хоста: запоминаем и пускаем."""
        def missing_host_key(self, client, hostname, key):
            os.makedirs(os.path.dirname(known_hosts_path), exist_ok=True)
            client.get_host_keys().add(hostname, key.get_name(), key)
            client.save_host_keys(known_hosts_path)

    return TOFUPolicy()


def connect(profile, password=None, known_hosts=DEFAULT_KNOWN_HOSTS, timeout=10):
    """
    Возвращает подключённый paramiko.SSHClient.
    Бросает HostKeyChanged, если ключ хоста не совпал с запомненным.
    """
    import paramiko

    errors = profile.validate()
    if errors:
        raise ValueError("; ".join(errors))

    client = paramiko.SSHClient()
    if os.path.exists(known_hosts):
        client.load_host_keys(known_hosts)
    client.set_missing_host_key_policy(_tofu_policy(known_hosts))

    kwargs = dict(hostname=profile.host, port=int(profile.port), username=profile.user,
                  timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    if profile.key_path:
        kwargs.update(key_filename=os.path.expanduser(profile.key_path),
                      look_for_keys=False, allow_agent=False)
    if password:
        kwargs["password"] = password
    try:
        client.connect(**kwargs)
    except paramiko.BadHostKeyException as e:
        client.close()
        raise HostKeyChanged(
            f"Ключ хоста {profile.host} изменился! Возможна подмена машины. "
            f"Если робот переустанавливали — удали его строку из {known_hosts}.") from e
    return client
