import docker
import logging
import platform
import io
import tarfile
import time
import os

logging.basicConfig(level=logging.INFO)

class RosContainerManager:
    def __init__(self):
        self.client = None
        self.container = None
        self.image_name = "osrf/ros:humble-desktop"
        self.container_name = "ros2_orchestrator_session"
        
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

        if platform.system() == 'Windows':
            environment['DISPLAY'] = 'host.docker.internal:0.0'
        else:
            environment['DISPLAY'] = ':0'
            volumes_map['/tmp/.X11-unix'] = {'bind': '/tmp/.X11-unix', 'mode': 'rw'}
            volumes_map['/dev'] = {'bind': '/dev', 'mode': 'rw'}

        self.container = self.client.containers.run(
            self.image_name,
            command="bash -c 'sleep infinity'",
            name=self.container_name,
            detach=True,
            tty=True,
            environment=environment,
            volumes=volumes_map,
            network_mode='host',
            privileged=True
        )
        
        # Turtlesim нужен для тестов
        self.container.exec_run("apt-get update && apt-get install -y ros-humble-turtlesim", detach=True)
        
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
            sys_callback("🔨 Building C++ project...")
            
            build_cmd = (
                "bash -c 'source /opt/ros/humble/setup.bash && "
                "cd /root/ros2_ws && "
                "colcon build --packages-select cpp_blueprints_pkg --event-handlers console_direct+'"
            )
            # Логи сборки отправляем в SYSTEM LOG
            build_stream = self.container.exec_run(build_cmd, stream=True)
            for line in build_stream.output:
                sys_callback(line.decode('utf-8', errors='replace').strip())
            
            sys_callback("✅ Build phase finished.")
        else:
            sys_callback("🐍 Python project detected. Skipping build.")

        # 2. ЗАПУСК ROS
        sys_callback("🚀 Launching ROS 2... (Switch to ROS Output tab)")
        
        launch_cmd = (
            "bash -c 'source /opt/ros/humble/setup.bash && "
            "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi && "
            "export PYTHONUNBUFFERED=1 && "
            "export RCUTILS_COLORIZED_OUTPUT=1 && "
            "ros2 launch /root/ros2_ws/src/user_project/launch/project_launch.py'"
        )
        
        # Логи работы отправляем в ROS LOG
        launch_stream = self.container.exec_run(launch_cmd, stream=True, tty=True)
        
        for line in launch_stream.output:
            text = line.decode('utf-8', errors='replace').strip()
            ros_callback(text)


    def ensure_image(self, cb): return # (уже есть выше)