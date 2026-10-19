"""
RemoteSession: the single entry point for the UI: connection, files, sync, build.

Layout on the remote side (a normal colcon workspace):
    <remote_workspace>/src/<pkg>/   <- <project>/src is synced here
    <remote_workspace>/build, install, log  <- colcon; sync never touches them
"""
import os

from core.remote.profile import connect as _connect
from core.remote.providers import SSHProvider, SSHContainerProvider
from core.remote.runner import RemoteRunner
from core.remote.sync import sync_tree
from core.generators.build_generator import package_name_from_dir
from core.i18n import t


def package_name_for(project_path):
    """Same rule as the CMakeLists/package.xml generator."""
    return package_name_from_dir(project_path)


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

    # ---------- connection ----------

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
            raise NotConnected(t("rp.not_connected_error"))

    # ---------- files ----------

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

    # ---------- operations ----------

    def sync(self, on_line=None, dry_run=False):
        self._require()
        report = sync_tree(self.local_src, self._host, self.remote_src,
                           dry_run=dry_run, on_progress=on_line)
        if on_line:
            on_line(f"Sync → {self.profile.display()}:{self.remote_src}  {report.summary()}")
            for rel, why in report.conflicts:
                on_line(t("sync.conflict_line", path=rel, reason=why))
        return report

    def build(self, on_line, packages=None):
        self._require()
        return self.runner.build(on_line, packages or [self.package])

    def cancel(self):
        return bool(self.runner and self.runner.cancel())
