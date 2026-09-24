import docker

# Внятная ошибка вместо загадочного AttributeError ниже: пакет docker может
# отсутствовать, либо 'docker' может оказаться посторонним каталогом в
# sys.path (каталог с Dockerfile базового образа поэтому и назван
# docker_image, а не docker).
if not hasattr(docker, "errors"):
    raise ImportError(
        "Пакет 'docker' не установлен или затенён посторонним каталогом с таким "
        "именем. Установи зависимости: python -m pip install -r requirements.txt")
import logging
import platform
import io
import tarfile
import time
import os

from core.apt_safe import (
    apt_cmd, apt_snippet, missing_check_snippet, validate_packages,
    InvalidPackageName,
)

logging.basicConfig(level=logging.INFO)

# Базовый образ студии с запечёнными пакетами (docker_image/Dockerfile.studio).
# Собирается локально скриптом docker_image/build_studio_image.*; в реестре его нет.
STUDIO_IMAGE = "blueprint-studio:humble"
FALLBACK_IMAGE = "osrf/ros:humble-desktop"


def pick_base_image(image_exists):
    """
    image_exists: callable(tag) -> bool.
    Студийный образ, если он собран; иначе публичный osrf (тогда пакеты
    сессии доставит apt_safe в фоне).
    """
    try:
        if image_exists(STUDIO_IMAGE):
            return STUDIO_IMAGE
    except Exception:
        pass
    return FALLBACK_IMAGE

class RosContainerManager:
    def __init__(self, base_url=None):
        self.client = None
        self.container = None
        self.image_name = FALLBACK_IMAGE
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

        self.image_name = pick_base_image(self._image_exists)

    def _image_exists(self, tag):
        try:
            self.client.images.get(tag)
            return True
        except docker.errors.ImageNotFound:
            return False

    def ensure_image(self, status_callback=None):
        def log(m):
            if status_callback: status_callback(m)
        if self._image_exists(self.image_name):
            log(f"Образ найден: {self.image_name}")
        elif self.image_name == STUDIO_IMAGE:
            # Собирается только локально — тянуть неоткуда.
            log(f"{STUDIO_IMAGE} не найден, откат на {FALLBACK_IMAGE}")
            self.image_name = FALLBACK_IMAGE
            self.client.images.pull(self.image_name)
        else:
            log("Загрузка образа...")
            self.client.images.pull(self.image_name)
        if self.image_name == FALLBACK_IMAGE:
            log(f"💡 Совет: собери {STUDIO_IMAGE} (docker_image/build_studio_image.*) — "
                "тогда пакеты не будут доустанавливаться при каждой новой сессии.")

    def start_session(self, project_path, status_callback=None, force_recreate=False):
        def log(m):
            if status_callback: status_callback(m)

        project_image = f"blueprint-{os.path.basename(project_path).lower()}:latest"
        try:
            self.client.images.get(project_image)
            self.image_name = project_image
        except docker.errors.ImageNotFound:
            pass

        # Идемпотентность: если контейнер уже жив и создан из того же образа —
        # переиспользуем его, не сносим. Это защищает от потери apt-пакетов,
        # запущенных вручную bringup-сессий и т.д. при повторном Run.
        try:
            existing = self.client.containers.get(self.container_name)
            existing.reload()
            same_image = existing.attrs.get('Config', {}).get('Image') in (
                self.image_name, project_image)
            if existing.status == "running" and same_image and not force_recreate:
                self.container = existing
                log(f"♻️  Reusing existing live session (container already running).")
                return
            else:
                existing.remove(force=True)
        except docker.errors.NotFound:
            pass
        except Exception as e:
            log(f"WARN: couldn't inspect/remove old container: {e}")

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
            
            run_kwargs['ports'] = {'8765/tcp': 8765, '9877/udp': 9877}
        else:
            environment['DISPLAY'] = ':0'
            volumes_map['/tmp/.X11-unix'] = {'bind': '/tmp/.X11-unix', 'mode': 'rw'}
            volumes_map['/dev'] = {'bind': '/dev', 'mode': 'rw'}
            run_kwargs['network_mode'] = 'host'

        self.container = self.client.containers.run(self.image_name, **run_kwargs)
        
        
        self._install_session_packages(log)
        if status_callback: status_callback("Session Started (Logs Unbuffered).")


    # Пакеты, которые студии нужны в любой сессии. В образе
    # blueprint-studio:humble они уже запечены — тогда apt не зовётся вовсе.
    SESSION_PACKAGES = ["ros-humble-turtlesim", "ros-humble-foxglove-bridge"]

    def _install_session_packages(self, log=None):
        """
        Ставит в фоне только НЕДОСТАЮЩИЕ пакеты сессии. Маркер
        /tmp/session_setup_done выставляется в любом случае.
        Возвращает список пакетов, которые пришлось ставить.
        """
        chk = self.container.exec_run(
            ["bash", "-c", missing_check_snippet(self.SESSION_PACKAGES)])
        missing = (chk.output or b"").decode(errors="ignore").split()
        missing = [m for m in missing if m in self.SESSION_PACKAGES]

        if not missing:
            self.container.exec_run(["bash", "-c", "touch /tmp/session_setup_done"])
            if log: log("📦 Session packages already present — no apt needed.")
            return []

        if log: log(f"📦 Installing missing session packages in background: {' '.join(missing)}")
        self.container.exec_run(
            ["bash", "-c",
             f"({apt_snippet(missing)}) > /tmp/session_setup.log 2>&1; "
             "touch /tmp/session_setup_done"],
            detach=True)
        return missing

    def run_project_launch(self, sys_callback, ros_callback, extra_apt_packages=None):
        check_cmake = self.container.exec_run("test -f /root/ros2_ws/src/user_project/CMakeLists.txt")
        is_cpp = (check_cmake.exit_code == 0)

        if is_cpp:
            sys_callback("📦 Checking and installing system dependencies (rosdep)...")

            extra = []
            if extra_apt_packages:
                try:
                    extra = validate_packages(extra_apt_packages)
                    sys_callback(f"📦 Also installing (from dependency registry): {' '.join(extra)}")
                except InvalidPackageName as e:
                    sys_callback(f"⚠ Skipping invalid package names from registry: {e}")

            dep_cmd = ["bash", "-c",
                apt_snippet(extra, update=True) + "; "
                "for i in $(seq 1 15); do "
                "  rosdep install -y --from-paths /root/ros2_ws/src --ignore-src --rosdistro humble "
                "    > /tmp/rosdep_out.log 2>&1 && break; "
                "  echo \"[deps] rosdep failed (lock busy?), retry $i...\"; sleep 3; "
                "done; "
                "cat /tmp/rosdep_out.log"]

            dep_stream = self.container.exec_run(dep_cmd, stream=True)
            for line in dep_stream.output:
                sys_callback(line.decode('utf-8', errors='replace').strip())

            sys_callback("🔨 Building C++ project...")

            build_cmd = (
                "bash -c 'source /opt/ros/humble/setup.bash && "
                "cd /root/ros2_ws && "
                "colcon build --symlink-install --event-handlers console_direct+'"
            )
            build_stream = self.container.exec_run(build_cmd, stream=True)
            for line in build_stream.output:
                sys_callback(line.decode('utf-8', errors='replace').strip())

            sys_callback("✅ Build phase finished.")
        else:
            sys_callback("🐍 Python project detected. Skipping build.")

        # 2. ЗАПУСК ROS — этот блок у тебя уже есть ниже, НЕ трогай его
        sys_callback(" Launching ROS 2... (Switch to ROS Output tab)")
        launch_cmd = (
            "bash -c 'source /opt/ros/humble/setup.bash && "
            "if [ -d /root/ros2_ws/install ]; then source /root/ros2_ws/install/setup.bash; fi && "
            "export PYTHONUNBUFFERED=1 && "
            "export RCUTILS_COLORIZED_OUTPUT=1 && "
            "ros2 launch /root/ros2_ws/src/user_project/launch/project_launch.py'"
        )
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
            log(" foxglove_bridge ещё не установлен. Жду apt (фоновая установка сессии)...")
            inst = self.container.exec_run(apt_cmd(["ros-humble-foxglove-bridge"]))
            # Честная проверка результата, а не слепой рапорт
            recheck = self.container.exec_run(
                "bash -c \"test -d /opt/ros/humble/share/foxglove_bridge && echo yes || echo no\"")
            if b"no" in (recheck.output or b""):
                tail = (inst.output or b"")[-400:].decode(errors="ignore")
                raise RuntimeError(f"Установка foxglove_bridge не удалась. Хвост лога: {tail}")
            log(" foxglove_bridge установлен (проверено).")

    
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
            try:
                cmd = apt_cmd([pkg_name])
            except InvalidPackageName as e:
                if output_callback: output_callback(f"Error: {e}")
                return False
        else:
            cmd = f"pip install {pkg_name}"

        if output_callback: output_callback(f"Installing {pkg_name} via {pkg_type}...")
        
        result = self.container.exec_run(cmd, stream=True)
        for line in result.output:
            if output_callback:
                output_callback(line.decode('utf-8', errors='replace').strip())
        
        return True
    
    def rebuild_project_image(self, project_path, status_callback=None):
        """Собирает Docker-образ проекта из его Dockerfile."""
        def log(m):
            if status_callback: status_callback(m)
        dockerfile_path = os.path.join(project_path, "Dockerfile")
        if not os.path.exists(dockerfile_path):
            raise RuntimeError("No Dockerfile found for this project.")
        project_name = os.path.basename(project_path).lower()
        image_tag = f"blueprint-{project_name}:latest"
        log(f"🔨 Building project image {image_tag}...")
        base = pick_base_image(self._image_exists)
        log(f"   base image: {base}")
        self.client.images.build(path=project_path, dockerfile="Dockerfile",
                                 tag=image_tag, rm=True,
                                 buildargs={"BASE_IMAGE": base})
        log(f"✅ Image ready: {image_tag}")
        return image_tag