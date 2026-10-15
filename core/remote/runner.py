"""
Удалённая сборка и запуск: colcon build / ros2 run на целевой машине.

Отмена — главный подводный камень:
- закрыть SSH-канал без PTY недостаточно: colcon продолжит работать на роботе;
- `docker exec` без -t, если убить его клиента, НЕ убивает процесс внутри
  контейнера (известное поведение Docker).
Поэтому команда стартует в собственной группе процессов (setsid), первой
строкой печатает свой PGID, а отмена шлёт TERM, затем KILL всей группе —
тем же путём (внутри того же контейнера), что и запуск. Убиваются colcon,
make, gcc — всё дерево.

stderr сливается в stdout внутри скрипта (2>&1), поэтому ошибка компиляции
приходит в System Log целиком и в правильном порядке.
"""
import re
import shlex
import threading
import time

from core.remote.providers import exec_remote

_PGID_MARK = "__BP_PGID__"
_PGID_RE = re.compile(rf"^{_PGID_MARK} (\d+)$")


class PreflightError(RuntimeError):
    pass


def quote_argv(argv):
    return " ".join(shlex.quote(a) for a in argv)


class RemoteRunner:
    def __init__(self, ssh, profile, package_name=""):
        self.ssh = ssh
        self.profile = profile
        self.package_name = package_name
        self._lock = threading.Lock()
        self._running = {}          # имя -> pgid (фоновые процессы: ноды)
        self._current_pgid = None   # активная foreground-команда (сборка)
        self._current_channel = None

    # ---------- композиция команд (чистые функции) ----------

    def _prefix(self):
        if self.profile.container:
            return ["docker", "exec", "-i", self.profile.container]
        return []

    def _wrap(self, script):
        """Скрипт → строка для exec_command с учётом контейнера."""
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
                # colcon ищем ПОСЛЕ source: сборка тоже сначала делает source,
                # иначе проверка и сборка видят разный PATH
                f"( source {setup} >/dev/null 2>&1 && command -v colcon >/dev/null ) "
                f"|| {{ echo 'NO_COLCON'; exit 5; }}; "
                f"echo OK")

    def run_script(self, executable, args=()):
        if not self.package_name:
            raise ValueError("package_name не задан")
        argv = ["ros2", "run", self.package_name, executable] + list(args)
        return f"{self._ros_env()} && exec {quote_argv(argv)}"

    # ---------- исполнение ----------

    def preflight(self):
        """Проверяет окружение ДО сборки, с понятным текстом вместо трейсбека."""
        rc, out, err = exec_remote(self.ssh, self._wrap(self.preflight_script()))
        text = (out or b"").decode(errors="replace").strip()
        if rc == 0 and text.endswith("OK"):
            return
        where = f"в контейнере {self.profile.container}" if self.profile.container else "на хосте"
        msg = {
            "NO_WORKSPACE": f"нет каталога {self.profile.build_dir} {where}"
                            + (" — смонтирован ли он в контейнер (-v)?" if self.profile.container else ""),
            "NO_SRC": f"в {self.profile.build_dir} нет папки src — сначала сделай Sync",
            "NO_ROS": f"не найден {self.profile.ros_setup} {where}",
            "NO_COLCON": f"colcon не установлен {where}",
        }
        for key, m in msg.items():
            if key in text:
                raise PreflightError(m)
        detail = (err or b"").decode(errors="replace").strip()
        if "No such container" in detail:
            raise PreflightError(f"контейнер {self.profile.container} не найден или не запущен")
        raise PreflightError(f"проверка окружения не прошла: {text or detail or f'rc={rc}'}")

    def run_streaming(self, script, on_line, name=None):
        """
        Запускает скрипт, стримит объединённый stdout+stderr по строкам.
        Возвращает exit code. Отменяется через cancel() из другого потока.
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
                else:
                    time.sleep(0.02)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                emit(raw)
            if buf:
                emit(buf)
            return chan.recv_exit_status()
        finally:
            with self._lock:
                self._current_channel = None
                self._current_pgid = None
                if name:
                    self._running.pop(name, None)
            chan.close()

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
        """Останавливает текущую foreground-команду вместе со всем деревом."""
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
        """Фоновые процессы, которые реально живы на удалённой стороне."""
        with self._lock:
            items = dict(self._running)
        alive = {}
        for name, pgid in items.items():
            rc, _, _ = exec_remote(self.ssh, self._wrap(f"kill -0 -- -{pgid} 2>/dev/null"))
            if rc == 0:
                alive[name] = pgid
        return alive
