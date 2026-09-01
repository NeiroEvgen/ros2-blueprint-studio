import threading


class ProcessManager:
    def __init__(self, docker_manager):
        self.docker = docker_manager
        self.processes = {}  # node_name -> {'exec_id', 'language', 'package_name', 'entry'}
        self._lock = threading.Lock()

    # ---- запуск / остановка отдельной ноды ---------------------------

    def start_node(self, node_name, language, package_name, executable_or_path,
                   status_callback=None):
        """language: 'cpp' или 'python'. executable_or_path: имя executable
        для cpp (ros2 run pkg exe) или абсолютный путь к .py файлу."""
        def log(m):
            if status_callback: status_callback(m)

        with self._lock:
            self._kill_by_name(node_name)

            if language == "cpp":
                cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
                      f"source /root/ros2_ws/install/setup.bash 2>/dev/null; "
                      f"exec ros2 run {package_name} {executable_or_path}'")
            else:
                cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
                      f"exec python3 -u {executable_or_path}'")

            self.docker.container.exec_run(cmd, detach=True)
            self.processes[node_name] = {
                'language': language, 'package_name': package_name,
                'entry': executable_or_path,
            }
            log(f"▶ Started node '{node_name}'")

    def stop_node(self, node_name, status_callback=None):
        def log(m):
            if status_callback: status_callback(m)
        if node_name not in self.processes:
            return
        self._kill_by_name(node_name)
        log(f"⏹ Stopped node '{node_name}'")

    def _kill_by_name(self, node_name):
        info = self.processes.get(node_name)
        target = info['entry'].split('/')[-1] if info else node_name
        self.docker.container.exec_run(f"bash -c \"pkill -f '{target}' || true\"")

    def restart_node(self, node_name, status_callback=None, rebuild_cpp=False):
        """Пересобирает (если C++ и попросили) и перезапускает одну ноду."""
        def log(m):
            if status_callback: status_callback(m)
        info = self.processes.get(node_name)
        if not info:
            log(f"⚠ Node '{node_name}' is not tracked, cannot restart")
            return

        if info['language'] == "cpp" and rebuild_cpp:
            log(f"🔨 Rebuilding package '{info['package_name']}'...")
            build_cmd = (
                "bash -c 'source /opt/ros/humble/setup.bash && cd /root/ros2_ws && "
                f"colcon build --symlink-install --packages-select {info['package_name']}'")
            result = self.docker.container.exec_run(build_cmd)
            if result.exit_code != 0:
                log(f"❌ Build failed:\n{result.output.decode(errors='ignore')[-800:]}")
                return
            log("✅ Build finished.")

        self.start_node(node_name, info['language'], info['package_name'],
                        info['entry'], status_callback)

    # ---- lifecycle (заморозка штатным механизмом ROS 2) ---------------

    def set_lifecycle_state(self, node_name, target_state, status_callback=None):
        """target_state: 'configure' | 'activate' | 'deactivate' | 'cleanup' | 'shutdown'"""
        def log(m):
            if status_callback: status_callback(m)
        cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
              f"source /root/ros2_ws/install/setup.bash 2>/dev/null; "
              f"ros2 lifecycle set /{node_name} {target_state}'")
        result = self.docker.container.exec_run(cmd)
        output = result.output.decode(errors='ignore').strip()
        if result.exit_code == 0:
            log(f"🧊 '{node_name}' -> {target_state}: {output}")
        else:
            log(f"⚠ Lifecycle transition failed for '{node_name}': {output}")

    def get_lifecycle_state(self, node_name):
        cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
              f"ros2 lifecycle get /{node_name}'")
        result = self.docker.container.exec_run(cmd)
        return result.output.decode(errors='ignore').strip()

    def list_running(self):
        return list(self.processes.keys())