import os
import shutil

class ExportManager:
    @staticmethod
    def export_to_portable_package(project_path, scripts, include_gui=True):
        """
        Генерирует папку docker_export.
        Автоматически определяет C++ ноды, создает для них пакет и компилирует.
        """
        export_dir = os.path.join(project_path, "docker_export")
        
        # Структура папок внутри контейнера
        # /ros2_ws/src/cpp_pkg (для C++)
        # /ros2_ws/src/scripts (для Python)
        
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

        # 3. ОБРАБОТКА C++ (Если есть)
        if cpp_scripts:
            os.makedirs(local_src_cpp)
            
            # Сохраняем .cpp файлы
            for s in cpp_scripts:
                fname = s['filename']
                with open(os.path.join(local_src_cpp, fname), 'w', encoding='utf-8', newline='\n') as f:
                    f.write(s['code'])
            
            # Генерируем CMakeLists.txt и package.xml
            cmake_content = ExportManager._generate_cmake("cpp_nodes_pkg", cpp_scripts)
            package_xml = ExportManager._generate_package_xml("cpp_nodes_pkg")
            
            # Кладем их в корень пакета (на уровень выше src)
            pkg_root = os.path.dirname(local_src_cpp)
            with open(os.path.join(pkg_root, "CMakeLists.txt"), 'w', newline='\n') as f:
                f.write(cmake_content)
            with open(os.path.join(pkg_root, "package.xml"), 'w', newline='\n') as f:
                f.write(package_xml)

        # 4. ГЕНЕРИРУЕМ DOCKERFILE
        base_image = "osrf/ros:humble-desktop" if include_gui else "ros:humble-ros-core"
        
        # Если есть C++, добавляем шаг сборки (colcon build)
        build_step = ""
        if cpp_scripts:
            build_step = """
# Copy C++ package
COPY cpp_pkg/ /ros2_ws/src/cpp_nodes_pkg/

# Build C++ nodes
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select cpp_nodes_pkg
"""

        dockerfile_content = f"""
FROM {base_image}

# Install dependencies and utilities
# ОБНОВЛЕНО: Добавлены build-essential, cmake и colcon для сборки C++
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    dos2unix \\
    build-essential \\
    cmake \\
    python3-colcon-common-extensions

# Setup Workspace
WORKDIR /ros2_ws

# Copy Python scripts
COPY py_scripts/ /ros2_ws/src/scripts/

{build_step}

# Copy starter script
COPY start.sh /start.sh
RUN dos2unix /start.sh && chmod +x /start.sh

CMD ["/start.sh"]
"""
        with open(os.path.join(export_dir, "Dockerfile"), 'w', newline='\n') as f:
            f.write(dockerfile_content)

        # 5. ГЕНЕРИРУЕМ START.SH
        # Он должен сорсить не только ROS, но и наш workspace (install/setup.bash)
        start_script = "#!/bin/bash\n"
        start_script += "source /opt/ros/humble/setup.bash\n"
        
        # Если была сборка, подключаем локальный оверлей
        if cpp_scripts:
            start_script += "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi\n\n"
        
        start_script += "echo '--- STARTING NODES ---'\n\n"

        # Запуск Python
        for s in py_scripts:
            start_script += f"python3 /ros2_ws/src/scripts/{s['filename']} &\n"
            
        # Запуск C++ (через ros2 run)
        for s in cpp_scripts:
            # Имя ноды = имя файла без .cpp
            node_exec = s['filename'].replace('.cpp', '')
            start_script += f"ros2 run cpp_nodes_pkg {node_exec} &\n"

        # Вечный цикл
        start_script += "\necho 'All nodes started. Container is alive.'\n"
        start_script += "tail -f /dev/null"

        with open(os.path.join(export_dir, "start.sh"), 'w', newline='\n') as f:
            f.write(start_script)

        # 6. DOCKER-COMPOSE
        compose_content = f"""
version: '3'
services:
  ros_project:
    build: .
    network_mode: host

    privileged: true
    environment:
      - DISPLAY={'${DISPLAY}' if include_gui else ''}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix

      - /dev:/dev
    stdin_open: true
    tty: true
"""
        with open(os.path.join(export_dir, "docker-compose.yaml"), 'w', newline='\n') as f:
            f.write(compose_content)

        return export_dir

    @staticmethod
    def _generate_cmake(pkg_name, nodes):
        # Стандартный CMake для ROS 2
        content = f"""cmake_minimum_required(VERSION 3.8)
project({pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Добавь другие пакеты сюда, если нужно

"""
        for node in nodes:
            exec_name = node['filename'].replace('.cpp', '')
            src_file = f"src/{node['filename']}"
            content += f"""
add_executable({exec_name} {src_file})
ament_target_dependencies({exec_name} rclcpp std_msgs geometry_msgs)
install(TARGETS {exec_name} DESTINATION lib/${{PROJECT_NAME}})
"""
        content += "\nament_package()\n"
        return content

    @staticmethod
    def _generate_package_xml(pkg_name):
        return f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{pkg_name}</name>
  <version>0.0.0</version>
  <description>Auto-generated C++ nodes</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>"""