"""
RemoteSession — одна точка входа для UI: подключение, файлы, sync, сборка.

Раскладка на удалённой стороне (обычный colcon workspace):
    <remote_workspace>/src/<pkg>/   ← сюда синхронизируется <project>/src
    <remote_workspace>/build, install, log  ← colcon, sync их не трогает
"""
import os
import re

from core.remote.profile import connect as _connect
from core.remote.providers import SSHProvider, SSHContainerProvider
from core.remote.runner import RemoteRunner
from core.remote.sync import sync_tree


def package_name_for(project_path):
    """То же правило, что у генератора CMakeLists/package.xml."""
    return re.sub(r"[^a-z0-9_]", "_", os.path.basename(os.path.normpath(project_path)).lower())


class NotConnected(RuntimeError):
    pass


class RemoteSession:
    def __init__(self, project_path, profile, known_hosts=None):
        self.project_path = project_path
        self.profile = profile
        self.known_hosts = known_hosts
        self.package = package_name_for(project_path)
        self.ssh = None
        self._host = None
        self._container = None
        self.runner = None

    # ---------- подключение ----------

    @property
    def connected(self):
        t = self.ssh.get_transport() if self.ssh else None
        return bool(t and t.is_active())

    def connect(self, password=None):
        kw = {"known_hosts": self.known_hosts} if self.known_hosts else {}
        self.ssh = _connect(self.profile, password=password, **kw)
        self._host = SSHProvider(self.ssh, label=self.profile.display())
        self._container = (SSHContainerProvider(self.ssh, self.profile.container)
                           if self.profile.container else None)
        self.runner = RemoteRunner(self.ssh, self.profile, self.package)

    def disconnect(self):
        if self._host:
            self._host.close()
        if self.ssh:
            self.ssh.close()
        self.ssh = self._host = self._container = self.runner = None

    def _require(self):
        if not self.connected:
            raise NotConnected("нет подключения к удалённой цели")

    # ---------- файлы ----------

    @property
    def local_src(self):
        return os.path.join(self.project_path, "src")

    @property
    def remote_src(self):
        return f"{self.profile.remote_workspace.rstrip('/')}/src/{self.package}"

    def host_provider(self):
        self._require()
        return self._host

    def container_provider(self):
        self._require()
        return self._container

    # ---------- операции ----------

    def sync(self, on_line=None, dry_run=False):
        self._require()
        report = sync_tree(self.local_src, self._host, self.remote_src,
                           dry_run=dry_run, on_progress=on_line)
        if on_line:
            on_line(f"Sync → {self.profile.display()}:{self.remote_src}  {report.summary()}")
            for rel, why in report.conflicts:
                on_line(f"⚠ конфликт {rel}: {why}")
        return report

    def build(self, on_line, packages=None):
        self._require()
        return self.runner.build(on_line, packages or [self.package])

    def cancel(self):
        return bool(self.runner and self.runner.cancel())
