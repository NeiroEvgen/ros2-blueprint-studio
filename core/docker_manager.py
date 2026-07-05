import docker
import logging
import platform
import io
import tarfile
import time
import os

logging.basicConfig(level=logging.INFO)

class RosContainerManager:
    def __init__(self, base_url=None):
        self.client = None
        self.container = None
        self.image_name = "osrf/ros:humble-desktop"
        self.container_name = "ros2_orchestrator_session"
        self.extra_env = {}   # v0.6.0: env из конфига группы 'main'

        if base_url:
            # Удалённый таргет: base_url='ssh://user@host' (docker SDK сам ходит по SSH)
            self.client = docker.DockerClient(base_url=base_url)
            self.client.ping()
        else:
            try:
                self.client = docker.DockerClient(base_url='npipe:////./pipe/docker_engine')
                self.client.ping()
            except Exception:
                try:
                    self.client = docker.from_env()
                    self.client.ping()
                except docker.errors.DockerException as e:
                    raise ConnectionError(f"Ошибка Docker: {e}")

    def ensure_image(self, status_callback=None):
        try:
            self.client.images.get(self.image_name)
            if status_callback: status_callback(f"Образ найден.")
        except docker.errors.ImageNotFound:
            if status_callback: status_callback(f"Загрузка образа...")
            self.client.images.pull(self.image_name)

    def start_session(self, project_path, status_callback=None):
        """
        Запускает контейнер с монтированием папки проекта.
        """
        try:
            old = self.client.containers.get(self.container_name)
            old.stop(); old.remove()
        except: pass

        abs_src = os.path.abspath(os.path.join(project_path, "src"))
        target_dir = "/root/ros2_ws/src/user_project"

        volumes_map = {
            abs_src: {'bind': target_dir, 'mode': 'rw'}
        }
        
        # === ВАЖНО: Глобальные переменные для логов ===
        environment = {
            "PYTHONUNBUFFERED": "1",          # Питон пишет сразу
            "RCUTILS_LOGGING_BUFFERED_STREAM": "0", # ROS пишет сразу
            "RCUTILS_COLORIZED_OUTPUT": "1"   # Цветной вывод
        }
        # v0.6.0: глобальные ROS env + DDS из конфига группы 'main'
        environment.update(getattr(self, 'extra_env', {}) or {})

        run_kwargs = dict(
            command="bash -c 'sleep infinity'",
            name=self.container_name,
            detach=True,
            tty=True,
            environment=environment,
            volumes=volumes_map,
            privileged=True
        )

        if platform.system() == 'Windows':
            environment['DISPLAY'] = 'host.docker.internal:0.0'
            
            run_kwargs['ports'] = {'8765/tcp': 8765}
        else:
            environment['DISPLAY'] = ':0'
            volumes_map['/tmp/.X11-unix'] = {'bind': '/tmp/.X11-unix', 'mode': 'rw'}
            volumes_map['/dev'] = {'bind': '/dev', 'mode': 'rw'}
            run_kwargs['network_mode'] = 'host'

        self.container = self.client.containers.run(self.image_name, **run_kwargs)
        
        
        self.container.exec_run(
            "bash -c \"apt-get update && apt-get install -y "
            "ros-humble-turtlesim ros-humble-foxglove-bridge "
            "> /tmp/session_setup.log 2>&1\"",
            detach=True)
        if status_callback: status_callback("Session Started (Logs Unbuffered).")

    def run_project_launch(self, sys_callback, ros_callback):
        """
        sys_callback: для сообщений о сборке и статусе.
        ros_callback: ТОЛЬКО для вывода запущенных нод.
        """
        
        # 1. ПРОВЕРКА C++
        check_cmake = self.container.exec_run("test -f /root/ros2_ws/src/user_project/CMakeLists.txt")
        is_cpp = (check_cmake.exit_code == 0)

        if is_cpp:
            # === НОВЫЙ БЛОК: УСТАНОВКА ЗАВИСИМОСТЕЙ ===
            sys_callback("📦 Checking and installing system dependencies (rosdep)...")
            
            # Обновляем списки и ставим зависимости, которые прописаны в package.xml
            dep_cmd = (
                "bash -c 'apt-get update && "
                "rosdep install -y --from-paths /root/ros2_ws/src --ignore-src --rosdistro humble'"
            )
            
            # Запускаем установку и транслируем логи в системное окно
            dep_stream = self.container.exec_run(dep_cmd, stream=True)
            for line in dep_stream.output:
                sys_callback(line.decode('utf-8', errors='replace').strip())
            # =========================================

            sys_callback("🔨 Building C++ project...")
            
            build_cmd = (
                "bash -c 'source /opt/ros/humble/setup.bash && "
                "cd /root/ros2_ws && "
                "colcon build --symlink-install --event-handlers console_direct+'"
            )
            # Логи сборки отправляем в SYSTEM LOG
            build_stream = self.container.exec_run(build_cmd, stream=True)
            for line in build_stream.output:
                sys_callback(line.decode('utf-8', errors='replace').strip())
            
            sys_callback("✅ Build phase finished.")
        else:
            sys_callback("🐍 Python project detected. Skipping build.")

        # 2. ЗАПУСК ROS
        sys_callback(" Launching ROS 2... (Switch to ROS Output tab)")
        
        # Определяем команду запуска в зависимости от наличия setup.bash в install
        launch_cmd = (
            "bash -c 'source /opt/ros/humble/setup.bash && "
            "if [ -d /root/ros2_ws/install ]; then source /root/ros2_ws/install/setup.bash; fi && "
            "export PYTHONUNBUFFERED=1 && "
            "export RCUTILS_COLORIZED_OUTPUT=1 && "
            "ros2 launch /root/ros2_ws/src/user_project/launch/project_launch.py'"
        )
        
        # Логи работы отправляем в ROS LOG
        launch_cmd_full = launch_cmd[:-1] + " 2>&1'"
        launch_stream = self.container.exec_run(launch_cmd_full, stream=True, tty=True)

        got_output = False
        for line in launch_stream.output:
            got_output = True
            text = line.decode('utf-8', errors='replace').strip()
            if text:
                ros_callback(text)

        if not got_output:
            ros_callback(" ros2 launch завершился мгновенно без вывода.")
        

    def start_foxglove_bridge(self, status_callback=None):
        """Ставит (если нет) и запускает foxglove_bridge на ws://localhost:8765."""
        def log(msg):
            if status_callback: status_callback(msg)

        if not self.container:
            raise RuntimeError("Container session is not running. Press Run first.")

        # 1. Уже запущен? Не плодим дубликаты.
        check = self.container.exec_run(
            "bash -c \"pgrep -f '[f]oxglove_bridge' || true\"")
        if check.output and check.output.decode().strip():
            log(" Foxglove bridge уже запущен (ws://localhost:8765)")
            return

        # 2. Установлен ли пакет
        check_pkg = self.container.exec_run(
            "bash -c \"test -d /opt/ros/humble/share/foxglove_bridge && echo yes || echo no\"")
        if b"no" in (check_pkg.output or b""):
            log(" Устанавливаю ros-humble-foxglove-bridge (один раз)...")
            inst = self.container.exec_run(
                "bash -c \"apt-get update && apt-get install -y ros-humble-foxglove-bridge\"",
                stream=True)
            for chunk in inst.output:
                pass  # тихо ждём; можно логировать chunk при желании
            log(" foxglove_bridge установлен.")

        # СТАЛО (вариант с полным отвязыванием от exec-сессии):
        self.container.exec_run(
            "bash -c \"source /opt/ros/humble/setup.bash && "
            "setsid nohup ros2 run foxglove_bridge foxglove_bridge "
            "--ros-args -p port:=8765 > /tmp/foxglove.log 2>&1 < /dev/null &\"",
            detach=True)
        # Даём процессу секунду подняться и проверяем, что он жив
        import time
        time.sleep(1.5)
        alive = self.container.exec_run("pgrep -f foxglove_bridge")
        if not (alive.output and alive.output.decode().strip()):
            tail = self.container.exec_run("tail -5 /tmp/foxglove.log").output
            raise RuntimeError(f"Bridge не запустился. Лог: {(tail or b'').decode(errors='ignore')}")
        log("🦊 Foxglove bridge запущен: ws://localhost:8765")


    def list_containers(self):
        """Возвращает список всех контейнеров в системе."""
        return self.client.containers.list(all=True)

    def list_networks(self):
        """Возвращает список всех Docker-сетей."""
        return self.client.networks.list()

    def stop_container(self, container_id):
        try:
            container = self.client.containers.get(container_id)
            container.stop()
            return True
        except: return False

    def start_existing_container(self, container_id):
        try:
            container = self.client.containers.get(container_id)
            container.start()
            return True
        except: return False

    def remove_container(self, container_id):
        try:
            container = self.client.containers.get(container_id)
            container.remove(force=True)
            return True
        except: return False

    def install_package(self, pkg_name, pkg_type="apt", output_callback=None):
        """
        Устанавливает пакет в запущенный контейнер.
        pkg_type: "apt" или "pip"
        """
        if not self.container:
            if output_callback: output_callback("Error: No active ROS container session.")
            return False

        if pkg_type == "apt":
            cmd = f"bash -c 'apt-get update && apt-get install -y {pkg_name}'"
        else:
            cmd = f"pip install {pkg_name}"

        if output_callback: output_callback(f"Installing {pkg_name} via {pkg_type}...")
        
        result = self.container.exec_run(cmd, stream=True)
        for line in result.output:
            if output_callback:
                output_callback(line.decode('utf-8', errors='replace').strip())
        
        return True