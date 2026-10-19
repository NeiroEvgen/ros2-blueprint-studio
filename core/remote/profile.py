"""
Remote target profile and connecting to it.

The profile is stored in the project: <project>/.blueprint/remote.yaml.
It holds NO secrets: only host, user, key path. The password, if
needed, is passed to connect() at connection time and is never
stored, so remote.yaml is safe to commit.

Host keys use TOFU (trust on first use), like plain ssh: on the first
connection the fingerprint is stored in ~/.blueprint/known_hosts; if the key
changes, the connection is REFUSED. AutoAddPolicy does not fit here: it would
silently accept a substituted machine.
"""
import os
from dataclasses import dataclass, asdict

import yaml

from core.i18n import t

PROFILE_REL = os.path.join(".blueprint", "remote.yaml")
DEFAULT_KNOWN_HOSTS = os.path.join(os.path.expanduser("~"), ".blueprint", "known_hosts")

_ALLOWED_KEYS = {"host", "port", "user", "key_path", "remote_workspace",
                 "container", "container_workspace", "ros_setup"}


@dataclass
class RemoteProfile:
    host: str = ""
    port: int = 22
    user: str = ""
    key_path: str = ""                  # empty = ssh-agent / ~/.ssh
    remote_workspace: str = ""          # where the sources go on the host
    container: str = ""                 # empty = build directly on the host
    container_workspace: str = ""       # the same folder as seen inside the container
    ros_setup: str = "/opt/ros/humble/setup.bash"

    @property
    def enabled(self):
        return bool(self.host and self.user and self.remote_workspace)

    @property
    def build_dir(self):
        """Folder to run colcon in: inside the container or on the host."""
        if self.container:
            return self.container_workspace or self.remote_workspace
        return self.remote_workspace

    def display(self):
        return f"{self.user}@{self.host}" + (f":{self.port}" if self.port != 22 else "")

    def validate(self):
        errors = []
        if not self.host:
            errors.append(t("rp.err_no_host"))
        if not self.user:
            errors.append(t("rp.err_no_user"))
        if not self.remote_workspace:
            errors.append(t("rp.err_no_workspace"))
        elif not self.remote_workspace.startswith("/"):
            errors.append(t("rp.err_workspace_abs"))
        if not (0 < int(self.port) < 65536):
            errors.append(t("rp.err_port"))
        if self.container and self.container_workspace and not self.container_workspace.startswith("/"):
            errors.append(t("rp.err_container_workspace_abs"))
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
        """Called only for an UNKNOWN host: remember it and let it in."""
        def missing_host_key(self, client, hostname, key):
            os.makedirs(os.path.dirname(known_hosts_path), exist_ok=True)
            client.get_host_keys().add(hostname, key.get_name(), key)
            client.save_host_keys(known_hosts_path)

    return TOFUPolicy()


def connect(profile, password=None, known_hosts=DEFAULT_KNOWN_HOSTS, timeout=10):
    """
    Returns a connected paramiko.SSHClient.
    Raises HostKeyChanged if the host key does not match the stored one.
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
            t("rp.hostkey_changed", host=profile.host, path=known_hosts)) from e
    enable_keepalive(client)
    return client


def enable_keepalive(client, idle=10, interval=5, count=3):
    """
    A dead connection (robot powered off, Wi-Fi gone) is otherwise detected only by
    the TCP timeout, tens of minutes. SSH keepalive keeps NAT/routers happy, TCP keepalive
    drops the socket after about idle + interval*count seconds of silence from the other side.
    """
    import socket
    tr = client.get_transport()
    if tr is None:
        return
    tr.set_keepalive(idle)
    sock = getattr(tr, "sock", None)
    if sock is None or not hasattr(sock, "setsockopt"):
        return
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, "SIO_KEEPALIVE_VALS"):                        # Windows
            sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, idle * 1000, interval * 1000))
            return
        for opt, val in (("TCP_KEEPIDLE", idle), ("TCP_KEEPALIVE", idle),   # Linux / macOS
                         ("TCP_KEEPINTVL", interval), ("TCP_KEEPCNT", count)):
            if hasattr(socket, opt):
                sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, opt), val)
    except OSError:
        pass                     # keepalive is a safety net, not a condition for connecting
