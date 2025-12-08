import docker
import logging
import platform
import io
import tarfile
import time

logging.basicConfig(level=logging.INFO)

class RosContainerManager:
    def __init__(self):
        self.client = None
        self.container = None
        self.image_name = "osrf/ros:humble-desktop"
        self.container_name = "ros2_orchestrator_session"
        
        # Try connecting via named pipe (Windows default)
        try:
            self.client = docker.DockerClient(base_url='npipe:////./pipe/docker_engine')
            self.client.ping()
        except Exception:
            # Fallback to environment variables
            try:
                self.client = docker.from_env()
                self.client.ping() 
            except docker.errors.DockerException as e:
                raise ConnectionError(f"Docker Error: {e}")

    def ensure_image(self, status_callback=None):
        try:
            self.client.images.get(self.image_name)
            if status_callback: status_callback(f"Image found locally.")
        except docker.errors.ImageNotFound:
            if status_callback: status_callback(f"Pulling image (this may take a while)...")
            self.client.images.pull(self.image_name)

    def start_session(self, status_callback=None):
        try:
            old = self.client.containers.get(self.container_name)
            old.stop()
            old.remove()
        except docker.errors.NotFound:
            pass

        # === Graphics Setup ===
        environment = {}
        if platform.system() == 'Windows':
            # Special DNS name for Windows host
            environment['DISPLAY'] = 'host.docker.internal:0.0'
        else:
            # Linux X11 forwarding
            environment['DISPLAY'] = ':0'

        self.container = self.client.containers.run(
            self.image_name,
            command="bash",
            name=self.container_name,
            detach=True,
            tty=True,
            environment=environment,
            shm_size="512m"     
        )
        
        # Install TurtleSim if missing
        self.container.exec_run("apt-get update && apt-get install -y ros-humble-turtlesim")
        self.container.exec_run("mkdir -p /root/ros2_ws/src")
        
        if status_callback: status_callback("Session started (GUI Enabled).")

    def inject_and_run_script(self, filename, code, status_callback=None):
        """Injects and runs a Python script."""
        self._inject_file(f"/root/{filename}", code)
        
        cmd = (
            f"bash -c 'source /opt/ros/humble/setup.bash && "
            f"export RCUTILS_LOGGING_BUFFERED_STREAM=1 && "
            f"export RCUTILS_COLORIZED_OUTPUT=0 && "
            f"python3 /root/{filename} "
            f"> /proc/1/fd/1 2>&1'"
        )
        if status_callback: status_callback(f"Running Py: {filename}")
        self.container.exec_run(cmd, detach=True)

    def build_and_run_cpp_nodes(self, cpp_nodes_data, status_callback=None):
        if not cpp_nodes_data: return

        pkg_name = "cpp_blueprints_pkg"
        src_path = f"/root/ros2_ws/src/{pkg_name}"
        
        if status_callback: status_callback("Preparing C++ build...")

        self.container.exec_run(f"mkdir -p {src_path}/src")

        # 1. CMakeLists.txt
        cmake_content = self._generate_cmake(pkg_name, cpp_nodes_data)
        self._inject_file(f"{src_path}/CMakeLists.txt", cmake_content)
        
        # 2. package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{pkg_name}</name>
  <version>0.0.0</version>
  <description>Auto-generated blueprints</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend> 
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>"""
        self._inject_file(f"{src_path}/package.xml", package_xml)

        for node in cpp_nodes_data:
            self._inject_file(f"{src_path}/src/{node['filename']}", node['code'])

        if status_callback: status_callback("Compiling C++ (colcon build)...")
        
        build_cmd = (
            f"bash -c 'source /opt/ros/humble/setup.bash && "
            f"cd /root/ros2_ws && "
            f"colcon build --packages-select {pkg_name}'"
        )
        res = self.container.exec_run(build_cmd)
        
        if res.exit_code != 0:
            err_msg = res.output.decode('utf-8')
            if status_callback: status_callback(f"BUILD ERROR:\n{err_msg}")
            return

        if status_callback: status_callback("Build success. Starting nodes...")

        for node in cpp_nodes_data:
            exec_name = node['filename'].replace('.cpp', '')
            run_cmd = (
                f"bash -c 'source /opt/ros/humble/setup.bash && "
                f"source /root/ros2_ws/install/setup.bash && "
                f"export RCUTILS_LOGGING_BUFFERED_STREAM=0 && "
                f"export RCUTILS_COLORIZED_OUTPUT=0 && "
                f"ros2 run {pkg_name} {exec_name} "
                f"> /proc/1/fd/1 2>&1'"
            )
            self.container.exec_run(run_cmd, detach=True)
        time.sleep(2)

    def _generate_cmake(self, pkg_name, nodes):
        content = f"""cmake_minimum_required(VERSION 3.8)
project({pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
"""
        for node in nodes:
            exec_name = node['filename'].replace('.cpp', '')
            src_file = f"src/{node['filename']}"
            content += f"""
add_executable({exec_name} {src_file})
ament_target_dependencies({exec_name} rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
install(TARGETS {exec_name} DESTINATION lib/${{PROJECT_NAME}})
"""
        content += "\nament_package()\n"
        return content

    def _inject_file(self, path, content):
        stream = io.BytesIO()
        with tarfile.open(fileobj=stream, mode='w') as tar:
            encoded = content.encode('utf-8')
            info = tarfile.TarInfo(name=path.split('/')[-1])
            info.size = len(encoded)
            tar.addfile(info, io.BytesIO(encoded))
        stream.seek(0)
        dest_dir = "/".join(path.split('/')[:-1])
        self.container.put_archive(path=dest_dir, data=stream)