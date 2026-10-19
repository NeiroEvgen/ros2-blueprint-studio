import threading
import traceback
from PySide6 import QtCore
from core.i18n import t
from core.phase_timer import PhaseTimer

class DeployWorker(QtCore.QThread):
    sys_signal = QtCore.Signal(str)
    ros_signal = QtCore.Signal(str)
    finished_signal = QtCore.Signal()

    def __init__(self, manager, project_path, extra_apt_packages=None):
        super().__init__()
        self.manager = manager
        self.project_path = project_path
        self.extra_apt_packages = extra_apt_packages or set()
        self.is_running = True
        self.timer = PhaseTimer()

    def run(self):
        self.sys_signal.emit("--- DEPLOY START ---")
        outcome = "error"
        try:
            with self.timer.phase("image"):
                self.manager.ensure_image(lambda m: self.sys_signal.emit(m))
            with self.timer.phase("session"):
                self.manager.start_session(self.project_path, lambda m: self.sys_signal.emit(m))
            ran = self.manager.run_project_launch(
                sys_callback=lambda m: self.sys_signal.emit(m),
                ros_callback=lambda m: self.ros_signal.emit(m),
                extra_apt_packages=self.extra_apt_packages,
                timer=self.timer,
            )
            outcome = "build_failed" if ran is False else "ok"
        except Exception as e:
            err_str = str(e)
            if "Container is not running" in err_str or "Read timed out" in err_str:
                outcome = "stopped"
                self.sys_signal.emit(" ROS 2 Session Stopped.")
            else:
                self.sys_signal.emit(f" ERROR: {err_str}")
                traceback.print_exc()
        finally:
            self.timer.record(self.project_path, outcome)
            self.sys_signal.emit(t("timing.summary", parts=self.timer.summary(),
                                   total=f"{self.timer.total:.1f}"))
            self.finished_signal.emit()

    STOP_TIMEOUT = 5      # seconds, then SIGKILL; enough for ROS nodes to shut down cleanly

    def stop(self):
        """
        Does not block the UI: docker stop runs in its own thread. The worker finishes by itself
        when the output stream of the stopped container ends.
        """
        self.is_running = False
        container = getattr(self.manager, "container", None)
        if container is None:
            return None

        def _stop():
            try:
                container.stop(timeout=self.STOP_TIMEOUT)
            except Exception as e:
                self.sys_signal.emit(f"stop: {e}")
        th = threading.Thread(target=_stop, name="docker-stop", daemon=True)
        th.start()
        return th