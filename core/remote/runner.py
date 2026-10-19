"""
Remote build and run: colcon build / ros2 run on the target machine.

Cancelling is the main pitfall:
- closing an SSH channel without a PTY is not enough: colcon keeps running on the robot;
- killing the client of `docker exec` without -t does NOT kill the process inside
  the container (known Docker behaviour).
So the command starts in its own process group (setsid), prints its PGID as the
first line, and cancel sends TERM, then KILL to the whole group, the same way
(inside the same container) as the start. colcon, make, gcc: the whole tree
is killed.

stderr is merged into stdout inside the script (2>&1), so a compile error
reaches the System Log complete and in the right order.
"""
import re
import shlex
import threading
import time

from core.i18n import t
from core.remote.providers import exec_remote

_PGID_MARK = "__BP_PGID__"
_PGID_RE = re.compile(rf"^{_PGID_MARK} (\d+)$")


class PreflightError(RuntimeError):
    pass


class RemoteConnectionLost(ConnectionError):
    """The SSH connection died in the middle of a command (robot powered off, network gone)."""


def quote_argv(argv):
    return " ".join(shlex.quote(a) for a in argv)


class RemoteRunner:
    def __init__(self, ssh, profile, package_name=""):
        self.ssh = ssh
        self.profile = profile
        self.package_name = package_name
        self._lock = threading.Lock()
        self._running = {}          # name -> pgid (background processes: nodes)
        self._current_pgid = None   # active foreground command (build)
        self._current_channel = None

    # ---------- command composition (pure functions) ----------

    def _prefix(self):
        if self.profile.container:
            return ["docker", "exec", "-i", self.profile.container]
        return []

    def _wrap(self, script):
        """Script -> exec_command string, wrapped for the container if there is one."""
        return quote_argv(self._prefix() + ["bash", "-c", script])

    def _ros_env(self):
        ws = self.profile.build_dir
        setup = shlex.quote(self.profile.ros_setup)
        return (f"source {setup} && cd {shlex.quote(ws)} && "
                f"if [ -f install/setup.bash ]; then source install/setup.bash; fi")

    def build_script(self, packages=None):
        pk = ""
        if packages:
            pk = " --packages-select " + " ".join(shlex.quote(p) for p in packages)
        ws = self.profile.build_dir
        return (f"source {shlex.quote(self.profile.ros_setup)} && "
                f"cd {shlex.quote(ws)} && "
                f"colcon build --symlink-install --event-handlers console_direct+{pk}")

    def preflight_script(self):
        ws = shlex.quote(self.profile.build_dir)
        src = shlex.quote(f"{self.profile.build_dir}/src")
        setup = shlex.quote(self.profile.ros_setup)
        return (f"test -d {ws} || {{ echo 'NO_WORKSPACE'; exit 2; }}; "
                f"test -d {src} || {{ echo 'NO_SRC'; exit 3; }}; "
                f"test -f {setup} || {{ echo 'NO_ROS'; exit 4; }}; "
                # look for colcon AFTER source: the build sources first too,
                # otherwise the check and the build see a different PATH
                f"( source {setup} >/dev/null 2>&1 && command -v colcon >/dev/null ) "
                f"|| {{ echo 'NO_COLCON'; exit 5; }}; "
                f"echo OK")

    def run_script(self, executable, args=()):
        if not self.package_name:
            raise ValueError("package_name is not set")
        argv = ["ros2", "run", self.package_name, executable] + list(args)
        return f"{self._ros_env()} && exec {quote_argv(argv)}"

    # ---------- execution ----------

    def preflight(self):
        """Checks the environment BEFORE building, with a readable message instead of a traceback."""
        rc, out, err = exec_remote(self.ssh, self._wrap(self.preflight_script()))
        text = (out or b"").decode(errors="replace").strip()
        if rc == 0 and text.endswith("OK"):
            return
        where = t("rp.where_container", container=self.profile.container) if self.profile.container else t("rp.where_host")
        msg = {
            "NO_WORKSPACE": t("rp.pre_no_workspace", path=self.profile.build_dir, where=where)
                            + (t("rp.pre_mounted_hint") if self.profile.container else ""),
            "NO_SRC": t("rp.pre_no_src", path=self.profile.build_dir),
            "NO_ROS": t("rp.pre_no_ros", path=self.profile.ros_setup, where=where),
            "NO_COLCON": t("rp.pre_no_colcon", where=where),
        }
        for key, m in msg.items():
            if key in text:
                raise PreflightError(m)
        detail = (err or b"").decode(errors="replace").strip()
        if "No such container" in detail:
            raise PreflightError(t("rp.pre_no_container", container=self.profile.container))
        raise PreflightError(t("rp.pre_failed", detail=text or detail or f'rc={rc}'))

    def run_streaming(self, script, on_line, name=None):
        """
        Runs a script and streams merged stdout+stderr line by line.
        Returns the exit code. Cancelled via cancel() from another thread.
        """
        wrapped = self._wrap(
            f"exec setsid -w bash -c {shlex.quote('echo ' + _PGID_MARK + ' $$; ' + script)} 2>&1")
        chan = self.ssh.get_transport().open_session()
        chan.exec_command(wrapped)
        with self._lock:
            self._current_channel = chan

        buf = b""
        pgid = None

        def emit(raw):
            nonlocal pgid
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            if pgid is None:
                m = _PGID_RE.match(line)
                if m:
                    pgid = int(m.group(1))
                    with self._lock:
                        self._current_pgid = pgid
                        if name:
                            self._running[name] = pgid
                    return
            on_line(line)

        try:
            while True:
                if chan.recv_ready():
                    chunk = chan.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        emit(raw)
                elif chan.exit_status_ready():
                    while chan.recv_ready():
                        buf += chan.recv(65536)
                    break
                elif not self._transport_alive():
                    # the connection died but the channel is not closed yet: no exit status will come
                    raise RemoteConnectionLost(t("rp.connection_lost"))
                else:
                    time.sleep(0.02)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                emit(raw)
            if buf:
                emit(buf)
            rc = chan.recv_exit_status()
            if rc == -1 and not self._transport_alive():
                # channel closed without an exit status together with the connection: not "the build failed"
                raise RemoteConnectionLost(t("rp.connection_lost"))
            return rc
        finally:
            with self._lock:
                self._current_channel = None
                self._current_pgid = None
                if name:
                    self._running.pop(name, None)
            chan.close()

    def _transport_alive(self):
        tr = self.ssh.get_transport()
        return bool(tr and tr.is_active())

    def build(self, on_line, packages=None):
        self.preflight()
        return self.run_streaming(self.build_script(packages), on_line)

    def _kill_group(self, pgid, grace=3.0):
        kill_term = self._wrap(f"kill -TERM -- -{pgid} 2>/dev/null; true")
        exec_remote(self.ssh, kill_term)
        deadline = time.time() + grace
        alive = self._wrap(f"kill -0 -- -{pgid} 2>/dev/null")
        while time.time() < deadline:
            rc, _, _ = exec_remote(self.ssh, alive)
            if rc != 0:
                return True
            time.sleep(0.2)
        exec_remote(self.ssh, self._wrap(f"kill -KILL -- -{pgid} 2>/dev/null; true"))
        return False

    def cancel(self):
        """Stops the current foreground command with its whole process tree."""
        with self._lock:
            pgid = self._current_pgid
        if pgid is None:
            return False
        self._kill_group(pgid)
        return True

    def stop(self, name):
        with self._lock:
            pgid = self._running.get(name)
        if pgid is None:
            return False
        self._kill_group(pgid)
        return True

    def list_running(self):
        """Background processes that are really alive on the remote side."""
        with self._lock:
            items = dict(self._running)
        alive = {}
        for name, pgid in items.items():
            rc, _, _ = exec_remote(self.ssh, self._wrap(f"kill -0 -- -{pgid} 2>/dev/null"))
            if rc == 0:
                alive[name] = pgid
        return alive
