import os
import shutil
import platform

class ExportManager:
    @staticmethod
    def export_to_portable_package(project_path, scripts, include_gui=True):
        """
        Генерирует папку docker_export.
        """
        export_dir = os.path.join(project_path, "docker_export")
        
        local_src_py = os.path.join(export_dir, "py_scripts")
        local_src_cpp = os.path.join(export_dir, "cpp_pkg", "src")
        
        if os.path.exists(export_dir):
            shutil.rmtree(export_dir)
            
        os.makedirs(local_src_py)
        
        # 1. РАЗДЕЛЯЕМ СКРИПТЫ
        py_scripts = [s for s in scripts if s['language'] == 'python']
        cpp_scripts = [s for s in scripts if s['language'] == 'cpp']

        # 2. СОХРАНЯЕМ PYTHON
        for s in py_scripts:
            fname = s['filename']
            with open(os.path.join(local_src_py, fname), 'w', encoding='utf-8', newline='\n') as f:
                f.write(s['code'])

        # 3. ОБРАБОТКА C++
        if cpp_scripts:
            os.makedirs(local_src_cpp)
            
            for s in cpp_scripts:
                fname = s['filename']
                with open(os.path.join(local_src_cpp, fname), 'w', encoding='utf-8', newline='\n') as f:
                    f.write(s['code'])
            
            pkg_name = "cpp_nodes_pkg" 
            cmake_content = ExportManager._generate_cmake(pkg_name, cpp_scripts, include_gui)
            package_xml = ExportManager._generate_package_xml(pkg_name, include_gui)
            
            pkg_root = os.path.dirname(local_src_cpp)
            with open(os.path.join(pkg_root, "CMakeLists.txt"), 'w', newline='\n') as f:
                f.write(cmake_content)
            with open(os.path.join(pkg_root, "package.xml"), 'w', newline='\n') as f:
                f.write(package_xml)

        # 4. ГЕНЕРИРУЕМ DOCKERFILE
        base_image = "osrf/ros:humble-desktop" if include_gui else "ros:humble-ros-core"
        
        build_step = ""
        if cpp_scripts:
            build_step = """
COPY cpp_pkg/ /ros2_ws/src/cpp_nodes_pkg/
# Установка зависимостей (игнорируем src, ставим системные либы)
RUN . /opt/ros/humble/setup.sh && rosdep install --from-paths src --ignore-src -r -y
WORKDIR /ros2_ws
# Сборка в один поток для стабильности
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select cpp_nodes_pkg --symlink-install --parallel-workers 1 --executor sequential
"""

        dockerfile_content = f"""
FROM {base_image}
# Установка базовых утилит
RUN apt-get update && apt-get install -y python3-pip dos2unix build-essential cmake git python3-colcon-common-extensions python3-rosdep
# Инициализация rosdep (удаляем старый файл во избежание конфликтов)
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update
WORKDIR /ros2_ws
COPY py_scripts/ /ros2_ws/src/scripts/
{build_step}
COPY start.sh /start.sh
RUN dos2unix /start.sh && chmod +x /start.sh
CMD ["/start.sh"]
"""
        with open(os.path.join(export_dir, "Dockerfile"), 'w', newline='\n') as f:
            f.write(dockerfile_content)

        # 5. ГЕНЕРИРУЕМ START.SH
        start_script = "#!/bin/bash\n"
        start_script += "source /opt/ros/humble/setup.bash\n"
        if cpp_scripts:
            start_script += "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi\n\n"
        start_script += "echo '--- STARTING NODES ---'\n\n"
        for s in py_scripts:
            start_script += f"python3 /ros2_ws/src/scripts/{s['filename']} &\n"
        for s in cpp_scripts:
            node_exec = s['filename'].replace('.cpp', '')
            start_script += f"ros2 run cpp_nodes_pkg {node_exec} &\n"
        start_script += "\necho 'Container is alive.'\n tail -f /dev/null"

        with open(os.path.join(export_dir, "start.sh"), 'w', newline='\n') as f:
            f.write(start_script)

        # 6. DOCKER-COMPOSE
        display_val = os.environ.get('DISPLAY', ':0')
        
        if platform.system() == 'Windows':
            display_val = "host.docker.internal:0.0"

        env_display = f"- DISPLAY={display_val}" if include_gui else ""
        vol_x11 = "- /tmp/.X11-unix:/tmp/.X11-unix" if include_gui and platform.system() != 'Windows' else ""
        qt_fix = "- QT_X11_NO_MITSHM=1" if include_gui else ""

        compose_content = f"""
version: '3'
services:
  ros_project:
    build: .
    network_mode: host
    privileged: true
    environment:
      {env_display}
      {qt_fix}
    volumes:
      {vol_x11}
      - /dev:/dev
    stdin_open: true
    tty: true
"""
        with open(os.path.join(export_dir, "docker-compose.yaml"), 'w', newline='\n') as f:
            f.write(compose_content)

        return export_dir
    
    @staticmethod
    def _generate_cmake(pkg_name, nodes, include_gui):
        libs = ["rclcpp", "std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs", "tf2", "tf2_ros", "tf2_geometry_msgs"]
        if include_gui: libs.extend(["visualization_msgs", "turtlesim"])
        
        find_packages = "\n".join([f"find_package({lib} REQUIRED)" for lib in libs])
        dep_list = " ".join(libs)
        
        content = f"""cmake_minimum_required(VERSION 3.8)
project({pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
{find_packages}

"""
        for node in nodes:
            exec_name = node['filename'].replace('.cpp', '')
            src_file = f"src/{node['filename']}"
            content += f"""
add_executable({exec_name} {src_file})
ament_target_dependencies({exec_name} {dep_list})
install(TARGETS {exec_name} DESTINATION lib/${{PROJECT_NAME}})
"""
        content += "\nament_package()\n"
        return content

    @staticmethod
    def _generate_package_xml(pkg_name, include_gui):
        libs = ["rclcpp", "std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs", "tf2", "tf2_ros", "tf2_geometry_msgs"]
        if include_gui: libs.extend(["visualization_msgs", "turtlesim"])
        
        depends = "\n  ".join([f"<depend>{lib}</depend>" for lib in libs])
        
        # ИСПРАВЛЕНИЕ: email="user@example.com" - это валидный формат
        return f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{pkg_name}</name>
  <version>0.0.0</version>
  <description>Auto-generated C++ nodes</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  {depends}
  
  <export><build_type>ament_cmake</build_type></export>
</package>"""