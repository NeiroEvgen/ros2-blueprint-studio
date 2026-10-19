import docker

# A clear error instead of a puzzling AttributeError below: the docker package may
# be missing, or 'docker' may resolve to an unrelated folder on
# sys.path (that is why the base-image Dockerfile folder is called
# docker_image, not docker).
if not hasattr(docker, "errors"):
    raise ImportError(
        "Package 'docker' is not installed or is shadowed by a folder with the same "
        "name. Install the dependencies: python -m pip install -r requirements.txt")
import logging
import platform
import time
import os
import re
import hashlib
from core import colcon
from core.i18n import t

from core.apt_safe import (
    apt_cmd, apt_snippet, missing_check_snippet, validate_packages,
    InvalidPackageName,
)

logging.basicConfig(level=logging.INFO)

# Studio base image with pre-baked packages (docker_image/Dockerfile.studio).
# Built locally by docker_image/build_studio_image.*; it is not in any registry.
STUDIO_IMAGE = "blueprint-studio:humble"
FALLBACK_IMAGE = "osrf/ros:humble-desktop"


def pick_base_image(image_exists):
    """
    image_exists: callable(tag) -> bool.
    The studio image if it has been built; otherwise the public osrf image (session
    packages are then installed in the background by apt_safe).
    """
    try:
        if image_exists(STUDIO_IMAGE):
            return STUDIO_IMAGE
    except Exception:
        pass
    return FALLBACK_IMAGE

def project_image_tag(project_path):
    """
    Docker tag of the project image. The folder name can be anything ("My Robot", non-Latin), but a tag
    allows only [a-z0-9._-]: otherwise images.get raises APIError, not ImageNotFound, and Run breaks.
    """
    base = os.path.basename(os.path.normpath(project_path))
    name = re.sub(r"[^a-z0-9._-]+", "-", base.lower())
    name = re.sub(r"[._-]{2,}", "-", name).strip("._-")
    if not name:
        name = "project-" + hashlib.sha1(base.encode("utf-8")).hexdigest()[:8]
    return f"blueprint-{name}:latest"


DEPS_STAMP_PREFIX = "/root/ros2_ws/.studio_deps_"
PROJECT_LABEL = "studio.project_src"


class _NullTimer:
    """Stand-in when timing is not needed."""
    from contextlib import contextmanager

    @contextmanager
    def phase(self, name):
        yield

    def mark(self, name):
        pass


class RosContainerManager:
    def __init__(self, base_url=None):
        self.client = None
        self.container = None
        self.image_name = FALLBACK_IMAGE
        self.container_name = "ros2_orchestrator_session"
        self.extra_env = {}   # v0.6.0: env from the 'main' group config

        if base_url:
            # Remote target: base_url='ssh://user@host' (the docker SDK talks SSH itself)
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
                    raise ConnectionError(f"Docker error: {e}")

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
            log(t("docker.image_found", image=self.image_name))
        elif self.image_name == STUDIO_IMAGE:
            # Built only locally: nothing to pull it from.
            log(t("docker.studio_image_missing", image=STUDIO_IMAGE, fallback=FALLBACK_IMAGE))
            self.image_name = FALLBACK_IMAGE
            self.client.images.pull(self.image_name)
        else:
            log(t("docker.pulling"))
            self.client.images.pull(self.image_name)
        if self.image_name == FALLBACK_IMAGE:
            log(t("docker.studio_image_tip", image=STUDIO_IMAGE))

    def start_session(self, project_path, status_callback=None, force_recreate=False):
        def log(m):
            if status_callback: status_callback(m)

        project_image = project_image_tag(project_path)
        try:
            self.client.images.get(project_image)
            self.image_name = project_image
        except docker.errors.ImageNotFound:
            pass
        except docker.errors.APIError as e:
            log(f"WARN: project image {project_image}: {e}")

        abs_src = os.path.abspath(os.path.join(project_path, "src"))

        # Idempotent: a container of the same image and the SAME project is reused,
        # running or stopped (Stop button). This keeps apt/rosdep packages and
        # build/ and install/: the next Run installs nothing and builds incrementally.
        # The project is identified by a label set at creation (Docker Mounts paths differ on Windows).
        try:
            existing = self.client.containers.get(self.container_name)
            existing.reload()
            same_image = existing.attrs.get('Config', {}).get('Image') in (
                self.image_name, project_image)
            owner = (getattr(existing, "labels", None) or {}).get(PROJECT_LABEL)
            same_project = owner is None or owner == abs_src      # no label: an old container
            usable = same_image and same_project and not force_recreate
            if usable and existing.status == "running":
                self.container = existing
                log("Reusing existing live session (container already running).")
                return
            if usable and owner is not None and existing.status in ("exited", "created"):
                existing.start()
                self.container = existing
                log("Reusing stopped session (dependencies and build cache kept).")
                return
            existing.remove(force=True)
        except docker.errors.NotFound:
            pass
        except Exception as e:
            log(f"WARN: couldn't inspect/remove old container: {e}")

        target_dir = "/root/ros2_ws/src/user_project"

        volumes_map = {
            abs_src: {'bind': target_dir, 'mode': 'rw'}
        }
        
        # === Unbuffered output so logs arrive immediately ===
        environment = {
            "PYTHONUNBUFFERED": "1",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
            "RCUTILS_COLORIZED_OUTPUT": "1"
        }
        # v0.6.0: global ROS env + DDS from the 'main' group config
        environment.update(getattr(self, 'extra_env', {}) or {})

        run_kwargs = dict(
            command="bash -c 'sleep infinity'",
            name=self.container_name,
            detach=True,
            tty=True,
            environment=environment,
            volumes=volumes_map,
            privileged=True,
            labels={PROJECT_LABEL: abs_src},
            # PID 1 is docker-init: without it PID 1 is sleep, which ignores SIGTERM,
            # and every Stop waits for the full docker stop timeout (10 s)
            init=True,
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


    # Packages Studio needs in every session. In the
    # blueprint-studio:humble image they are pre-baked, so apt is not called at all.
    SESSION_PACKAGES = ["ros-humble-turtlesim", "ros-humble-foxglove-bridge"]

    def _install_session_packages(self, log=None):
        """
        Installs only the MISSING session packages, in the background. The
        /tmp/session_setup_done marker is set in any case.
        Returns the list of packages that had to be installed.
        """
        chk = self.container.exec_run(
            ["bash", "-c", missing_check_snippet(self.SESSION_PACKAGES)])
        missing = (chk.output or b"").decode(errors="ignore").split()
        missing = [m for m in missing if m in self.SESSION_PACKAGES]

        if not missing:
            self.container.exec_run(["bash", "-c", "touch /tmp/session_setup_done"])
            if log: log("Session packages already present — no apt needed.")
            return []

        if log: log(f"Installing missing session packages in background: {' '.join(missing)}")
        self.container.exec_run(
            ["bash", "-c",
             f"({apt_snippet(missing)}) > /tmp/session_setup.log 2>&1; "
             "touch /tmp/session_setup_done"],
            detach=True)
        return missing

    def _deps_stamp(self, extra_apt):
        """
        Path of the "dependencies already installed" stamp for the current set of package.xml
        and extra apt packages; None if the fingerprint could not be computed (then always install).
        The stamp lives in the container: a new container installs dependencies again.
        """
        try:
            r = self.container.exec_run(["bash", "-c",
                "find /root/ros2_ws/src -name package.xml | sort | xargs -r cat | sha256sum"])
            digest = (r.output or b"").decode("utf-8", errors="replace").split()[0]
        except Exception:
            return None
        if r.exit_code != 0 or len(digest) != 64:
            return None
        fp = hashlib.sha256((digest + "|" + " ".join(sorted(extra_apt or []))).encode()).hexdigest()[:16]
        return f"{DEPS_STAMP_PREFIX}{fp}"

    def run_project_launch(self, sys_callback, ros_callback, extra_apt_packages=None, timer=None):
        timer = timer or _NullTimer()
        check_cmake = self.container.exec_run("test -f /root/ros2_ws/src/user_project/CMakeLists.txt")
        is_cpp = (check_cmake.exit_code == 0)

        if is_cpp:
            sys_callback(t("build.check_deps"))

            extra = []
            if extra_apt_packages:
                try:
                    extra = validate_packages(extra_apt_packages)
                    sys_callback(t("build.extra_deps", packages=' '.join(extra)))
                except InvalidPackageName as e:
                    sys_callback(t("build.invalid_pkg", error=e))

            stamp = self._deps_stamp(extra)
            if stamp and self.container.exec_run(["test", "-f", stamp]).exit_code == 0:
                sys_callback(t("build.deps_cached"))
            else:
                mark = (f"rm -f {DEPS_STAMP_PREFIX}*; touch {stamp}" if stamp else "true")
                dep_cmd = ["bash", "-c",
                    apt_snippet(extra, update=True) + "; apt_rc=$?; ok=0; "
                    "for i in $(seq 1 15); do "
                    "  rosdep install -y --from-paths /root/ros2_ws/src --ignore-src --rosdistro humble "
                    "    > /tmp/rosdep_out.log 2>&1 && ok=1 && break; "
                    "  echo \"[deps] rosdep failed (lock busy?), retry $i...\"; sleep 3; "
                    "done; "
                    "cat /tmp/rosdep_out.log; "
                    f"if [ $apt_rc -eq 0 ] && [ $ok -eq 1 ]; then {mark}; echo '[deps] cache stamp saved'; "
                    "else echo \"[deps] cache stamp NOT saved (apt exit=$apt_rc, rosdep ok=$ok)\"; fi"]

                with timer.phase("deps"):
                    dep_stream = self.container.exec_run(dep_cmd, stream=True)
                    for text in colcon.iter_lines(dep_stream.output):
                        sys_callback(text)

            sys_callback(t("build.building"))

            build_cmd = ["bash", "-c", colcon.build_command() + "; echo __COLCON_EXIT=$?"]
            build_lines, exit_code = [], None
            with timer.phase("colcon"):
                build_stream = self.container.exec_run(build_cmd, stream=True)
                for text in colcon.iter_lines(build_stream.output):
                    m = re.match(r"__COLCON_EXIT=(\d+)", text)
                    if m:
                        exit_code = int(m.group(1))
                        continue
                    build_lines.append(text)
                    sys_callback(text)

            result = colcon.parse_output(build_lines)
            if exit_code not in (0, None) or not result.ok:
                sys_callback(t("build.failed_prefix", summary=result.summary()))
                sys_callback(t("build.aborted"))
                return False

            sys_callback(t("build.finished"))
        else:
            sys_callback(t("build.python_skip"))

        # 2. Run the project launch file
        sys_callback(t("launch.starting"))
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
            if not got_output:
                timer.mark("first_output")
            got_output = True
            text = line.decode('utf-8', errors='replace').strip()
            if text:
                ros_callback(text)

        if not got_output:
            ros_callback(t("launch.no_output"))
        

    def start_foxglove_bridge(self, status_callback=None):
        """Installs (if missing) and starts foxglove_bridge on ws://localhost:8765."""
        def log(msg):
            if status_callback: status_callback(msg)

        if not self.container:
            raise RuntimeError("Container session is not running. Press Run first.")

        # 1. Already running? Do not start duplicates.
        check = self.container.exec_run(
            "bash -c \"pgrep -f '[f]oxglove_bridge' || true\"")
        if check.output and check.output.decode().strip():
            log(t("foxglove.already_running"))
            return

        # 2. Is the package installed?
        check_pkg = self.container.exec_run(
            "bash -c \"test -d /opt/ros/humble/share/foxglove_bridge && echo yes || echo no\"")
        if b"no" in (check_pkg.output or b""):
            log(t("foxglove.waiting_apt"))
            inst = self.container.exec_run(apt_cmd(["ros-humble-foxglove-bridge"]))
            # check the actual result, not just the exit code
            recheck = self.container.exec_run(
                "bash -c \"test -d /opt/ros/humble/share/foxglove_bridge && echo yes || echo no\"")
            if b"no" in (recheck.output or b""):
                tail = (inst.output or b"")[-400:].decode(errors="ignore")
                raise RuntimeError(t("foxglove.install_failed", tail=tail))
            log(t("foxglove.installed"))

    
        self.container.exec_run(
            "bash -c \"source /opt/ros/humble/setup.bash && "
            "setsid nohup ros2 run foxglove_bridge foxglove_bridge "
            "--ros-args -p port:=8765 > /tmp/foxglove.log 2>&1 < /dev/null &\"",
            detach=True)
        # give the process a moment to start and check it is alive
        time.sleep(1.5)
        alive = self.container.exec_run("pgrep -f foxglove_bridge")
        if not (alive.output and alive.output.decode().strip()):
            tail = self.container.exec_run("tail -5 /tmp/foxglove.log").output
            raise RuntimeError(t("foxglove.not_started", log=(tail or b'').decode(errors='ignore')))
        log(t("foxglove.started"))


    def list_containers(self):
        """All containers on the system."""
        return self.client.containers.list(all=True)

    def list_networks(self):
        """All Docker networks."""
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
        Installs a package into the running container.
        pkg_type: "apt" or "pip"
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
        """Builds the project's Docker image from its Dockerfile."""
        def log(m):
            if status_callback: status_callback(m)
        dockerfile_path = os.path.join(project_path, "Dockerfile")
        if not os.path.exists(dockerfile_path):
            raise RuntimeError("No Dockerfile found for this project.")
        image_tag = project_image_tag(project_path)
        log(f"Building project image {image_tag}...")
        base = pick_base_image(self._image_exists)
        log(f"   base image: {base}")
        self.client.images.build(path=project_path, dockerfile="Dockerfile",
                                 tag=image_tag, rm=True,
                                 buildargs={"BASE_IMAGE": base})
        log(f"Image ready: {image_tag}")
        return image_tag