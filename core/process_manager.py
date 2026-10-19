import re
import shlex
import threading

from core.i18n import t

CONTAINER_PROJECT_DIR = "/root/ros2_ws/src/user_project"


def node_spec(type_, source_file, package_name, bound_file=None):
    """
    How the node is started in the session (same rule as launch_generator):
    C++: `ros2 run <pkg> <file stem>`, Python: `python3 -u <path in the container>`.
    """
    if not source_file:
        return None
    if "cpp" in (type_ or ""):
        return {"language": "cpp", "package_name": package_name,
                "entry": source_file.rsplit(".", 1)[0]}
    rel = bound_file or f"python/{source_file}"
    return {"language": "python", "package_name": package_name,
            "entry": f"{CONTAINER_PROJECT_DIR}/{rel}"}


def _kill_pattern(info):
    """Regex for pkill -f that matches only this node's process (not talker_2 for talker)."""
    if info["language"] == "cpp":
        return f"/lib/{re.escape(info['package_name'])}/{re.escape(info['entry'])}( |$)"
    return f"python3 -u {re.escape(info['entry'])}( |$)"


class ProcessManager:
    def __init__(self, docker_manager):
        self.docker = docker_manager
        self.processes = {}  # node_name -> {'language', 'package_name', 'entry'}
        self._lock = threading.Lock()

    def _container(self, log):
        container = getattr(self.docker, "container", None) if self.docker else None
        if container is None:
            log(t("log.no_session_for_node"))
        return container

    # ---- start / stop a single node ---------------------------

    def start_node(self, node_name, language, package_name, executable_or_path,
                   status_callback=None):
        """language: 'cpp' or 'python'. executable_or_path: executable name
        for cpp (ros2 run pkg exe) or the absolute path to the .py file."""
        def log(m):
            if status_callback: status_callback(m)

        container = self._container(log)
        if container is None:
            return False
        info = {'language': language, 'package_name': package_name,
                'entry': executable_or_path}
        with self._lock:
            self._kill(container, info)
            if language == "cpp":
                run = f"exec ros2 run {shlex.quote(package_name)} {shlex.quote(executable_or_path)}"
                cmd = ("source /opt/ros/humble/setup.bash && "
                       f"source /root/ros2_ws/install/setup.bash 2>/dev/null; {run}")
            else:
                cmd = ("source /opt/ros/humble/setup.bash && "
                       f"exec python3 -u {shlex.quote(executable_or_path)}")
            container.exec_run(["bash", "-c", cmd], detach=True)
            self.processes[node_name] = info
        log(f"Started node '{node_name}'")
        return True

    def stop_node(self, node_name, status_callback=None):
        def log(m):
            if status_callback: status_callback(m)
        info = self.processes.get(node_name)
        container = self._container(log) if info else None
        if container is None:
            return
        self._kill(container, info)
        log(f"Stopped node '{node_name}'")

    @staticmethod
    def _kill(container, info):
        pattern = shlex.quote(_kill_pattern(info))
        container.exec_run(["bash", "-c", f"pkill -f {pattern} || true"])

    def restart_node(self, node_name, status_callback=None, rebuild_cpp=False, spec=None):
        """
        Rebuilds (for C++ if asked) and restarts one node.
        spec: how the node is started (node_spec); needed for nodes started from the launch file,
        which ProcessManager did not start itself.
        """
        def log(m):
            if status_callback: status_callback(m)
        info = self.processes.get(node_name) or spec
        if not info:
            log(f"⚠ Node '{node_name}' is not tracked, cannot restart")
            return False
        container = self._container(log)
        if container is None:
            return False

        if info['language'] == "cpp" and rebuild_cpp:
            log(f"Rebuilding package '{info['package_name']}'...")
            build_cmd = ("source /opt/ros/humble/setup.bash && cd /root/ros2_ws && "
                         "colcon build --symlink-install --packages-select "
                         f"{shlex.quote(info['package_name'])}")
            result = container.exec_run(["bash", "-c", build_cmd])
            if result.exit_code != 0:
                log(f"Build failed:\n{result.output.decode(errors='ignore')[-800:]}")
                return False
            log("Build finished.")

        return self.start_node(node_name, info['language'], info['package_name'],
                               info['entry'], status_callback)

    # ---- lifecycle (freezing via the standard ROS 2 mechanism) ---------------

    def set_lifecycle_state(self, node_name, target_state, status_callback=None):
        """target_state: 'configure' | 'activate' | 'deactivate' | 'cleanup' | 'shutdown'"""
        def log(m):
            if status_callback: status_callback(m)
        container = self._container(log)
        if container is None:
            return False
        cmd = ("source /opt/ros/humble/setup.bash && "
               "source /root/ros2_ws/install/setup.bash 2>/dev/null; "
               f"ros2 lifecycle set {shlex.quote('/' + node_name)} {shlex.quote(target_state)}")
        result = container.exec_run(["bash", "-c", cmd])
        output = result.output.decode(errors='ignore').strip()
        if result.exit_code == 0:
            log(f"'{node_name}' -> {target_state}: {output}")
            return True
        log(f"⚠ Lifecycle transition failed for '{node_name}': {output}")
        return False

    def get_lifecycle_state(self, node_name):
        container = getattr(self.docker, "container", None) if self.docker else None
        if container is None:
            return ""
        cmd = f"source /opt/ros/humble/setup.bash && ros2 lifecycle get {shlex.quote('/' + node_name)}"
        result = container.exec_run(["bash", "-c", cmd])
        return result.output.decode(errors='ignore').strip()

    def list_running(self):
        return list(self.processes.keys())
