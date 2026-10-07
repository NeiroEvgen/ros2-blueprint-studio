"""
Интерактивная консоль на удалённой цели: настоящий `ssh -t` в отдельном окне
терминала ОС. Встроенного эмулятора терминала в Studio нет и не нужно:
системный ssh сам спрашивает пароль, знает ssh-agent и ведёт себя как
обычная консоль (Ctrl+C, vim, tab-дополнение).

Пароль и ключ-фразу Studio сюда НЕ передаёт: их вводят в самом окне ssh.
Список known_hosts у системного ssh свой (не ~/.blueprint/known_hosts).

Значения host/user/container подставляются в командную строку, поэтому
проходят строгую проверку: на Windows строка уходит в cmd.exe, и символы
вроде & | > ^ превратились бы в выполнение чужих команд.
"""
import os
import re
import shutil
import subprocess

_HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]*$")      # имя, IPv4, простой IPv6
_USER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9._-]*$")
_CONTAINER_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")   # правило имён docker
_BAD_PATH_CHARS = set('&|<>^"%\r\n\x00')

# порядок поиска терминала в Linux
_LINUX_TERMINALS = [
    ("x-terminal-emulator", ["-e"]),
    ("gnome-terminal", ["--"]),
    ("konsole", ["-e"]),
    ("xfce4-terminal", ["-x"]),
    ("xterm", ["-e"]),
]


class TerminalError(RuntimeError):
    pass


def validate_target(profile):
    """Возвращает список ошибок; пустой — можно открывать."""
    errors = []
    if not profile.host or not _HOST_RE.match(profile.host):
        errors.append("некорректный хост (разрешены буквы, цифры, . - : _)")
    if not profile.user or not _USER_RE.match(profile.user):
        errors.append("некорректный пользователь")
    try:
        if not 0 < int(profile.port) < 65536:
            errors.append("некорректный порт")
    except (TypeError, ValueError):
        errors.append("некорректный порт")
    if profile.container and not _CONTAINER_RE.match(profile.container):
        errors.append("некорректное имя контейнера")
    key = (profile.key_path or "").strip()
    if key and any(c in _BAD_PATH_CHARS for c in key):
        errors.append('в пути к ключу недопустимые символы (& | < > ^ " %)')
    return errors


def ssh_argv(profile):
    """argv для системного ssh. Если задан контейнер — сразу заходим в него."""
    errors = validate_target(profile)
    if errors:
        raise TerminalError("; ".join(errors))
    argv = ["ssh", "-t", "-p", str(int(profile.port)),
            "-o", "ServerAliveInterval=30"]
    key = (profile.key_path or "").strip()
    if key:
        argv += ["-i", key]
    argv.append(f"{profile.user}@{profile.host}")
    if profile.container:
        # имя проверено регулярным выражением выше, кавычки не нужны
        argv.append(f"docker exec -it {profile.container} bash -l")
    return argv


def _windows_command(argv):
    """cmd /k оставляет окно открытым после выхода ssh — видно текст ошибки."""
    return "cmd.exe /k " + subprocess.list2cmdline(argv)


def open_terminal(profile, popen=subprocess.Popen, which=shutil.which, os_name=None):
    """
    Открывает новое окно терминала с ssh на цель из профиля.
    Возвращает строку-команду для показа в логе. popen/which/os_name
    подменяются в тестах.
    """
    argv = ssh_argv(profile)
    os_name = os_name or os.name
    shown = subprocess.list2cmdline(argv)

    if os_name == "nt":
        if not which("ssh"):
            raise TerminalError(
                "ssh.exe не найден. Включи «Клиент OpenSSH» в Параметры → Приложения → "
                "Дополнительные компоненты.")
        flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0x00000010)
        popen(_windows_command(argv), creationflags=flags)
        return shown

    if not which("ssh"):
        raise TerminalError("команда ssh не найдена (установи openssh-client)")
    for name, opt in _LINUX_TERMINALS:
        exe = which(name)
        if exe:
            popen([exe] + opt + argv)
            return shown
    raise TerminalError(
        "не найден эмулятор терминала (x-terminal-emulator, gnome-terminal, konsole, "
        "xfce4-terminal, xterm). Выполни вручную: " + shown)
