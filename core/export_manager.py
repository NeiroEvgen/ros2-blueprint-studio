import os
import shutil
import zipfile
import re

class ExportManager:
    @staticmethod
    def export_to_portable_package(project_path, include_gui=False):
        """
        Создает архив с готовым к запуску Docker-контейнером.
        Реализует паттерн Multi-stage (логически) для сборки ROS 2 workspace.
        Автоматически парсит и прописывает кастомные зависимости (includes).
        """
        export_dir = os.path.join(project_path, "export_docker")
        if os.path.exists(export_dir):
            shutil.rmtree(export_dir)
        os.makedirs(export_dir)

        # 1. Формируем правильную структуру рабочего пространства
        src_dir = os.path.join(project_path, "src")
        target_src = os.path.join(export_dir, "src", "user_project") 
        
        # Копируем исходники в подпапку, чтобы colcon корректно видел пакет
        shutil.copytree(src_dir, target_src)

        deps = set()
        
        # Ищем форматы пакетов с сообщениями: #include "nav_msgs/msg/odometry.hpp" -> nav_msgs
        pattern_msg = re.compile(r'#include\s+["<]([^/]+)/(msg|srv|action)/[^">]+[">]')
        # Ищем известные хардкорные ROS-либы (например, tf2_ros, cv_bridge)
        pattern_libs = re.compile(r'#include\s+["<](tf2_ros|tf2|tf2_geometry_msgs|image_transport|cv_bridge)/[^">]+[">]')

        # Сканируем скопированные исходники
        for root_dir, _, files in os.walk(target_src):
            for file in files:
                if file.endswith(('.cpp', '.hpp', '.h')):
                    with open(os.path.join(root_dir, file), 'r', encoding='utf-8') as f:
                        content = f.read()
                        
                        for match in pattern_msg.findall(content):
                            deps.add(match[0])
                        for match in pattern_libs.findall(content):
                            deps.add(match)
        
        # rclcpp обычно уже прописан жестко, но на всякий случай оставим
        deps.discard('rclcpp')

        # Если нашли кастомные инклуды — инжектим их в сборочные файлы!
        if deps:
            # А) Патчим package.xml для rosdep (чтобы Docker скачал пакеты через apt)
            package_xml_path = os.path.join(target_src, "package.xml")
            if os.path.exists(package_xml_path):
                with open(package_xml_path, 'r', encoding='utf-8') as f:
                    pkg_xml = f.read()
                
                for dep in deps:
                    # Если пакета еще нет в манифесте, вставляем его перед </package>
                    if f"<depend>{dep}</depend>" not in pkg_xml and f"<build_depend>{dep}</build_depend>" not in pkg_xml:
                        pkg_xml = pkg_xml.replace("</package>", f"  <depend>{dep}</depend>\n</package>")
                
                with open(package_xml_path, 'w', encoding='utf-8') as f:
                    f.write(pkg_xml)

            # Б) Патчим CMakeLists.txt для компилятора colcon
            cmake_path = os.path.join(target_src, "CMakeLists.txt")
            if os.path.exists(cmake_path):
                with open(cmake_path, 'r', encoding='utf-8') as f:
                    cmake_txt = f.read()
                
                find_pkgs_str = ""
                for dep in deps:
                    # Добавляем find_package, если его еще нет
                    if f"find_package({dep}" not in cmake_txt:
                        find_pkgs_str += f"find_package({dep} REQUIRED)\n"
                
                if find_pkgs_str:
                    # Вставляем новые пакеты после базового ament_cmake
                    if "find_package(ament_cmake REQUIRED)" in cmake_txt:
                        cmake_txt = cmake_txt.replace("find_package(ament_cmake REQUIRED)", f"find_package(ament_cmake REQUIRED)\n{find_pkgs_str}")
                    else:
                        cmake_txt = find_pkgs_str + "\n" + cmake_txt
                    
                    # Прописываем эти зависимости во все экзешники/ноды!
                    # Ищем "ament_target_dependencies(ИМЯ_НОДЫ текущие_зависимости)" и добавляем новые
                    extra_deps_str = " ".join(deps)
                    target_deps_pattern = re.compile(r'(ament_target_dependencies\s*\(\s*[^ \)]+)(.*?)\)')
                    cmake_txt = target_deps_pattern.sub(rf'\1\2 {extra_deps_str})', cmake_txt)

                with open(cmake_path, 'w', encoding='utf-8') as f:
                    f.write(cmake_txt)
        # =========================================================

        has_cmake = os.path.exists(os.path.join(target_src, "CMakeLists.txt"))
        
        # 2. Генерируем правильный Entrypoint
        launch_path = "/app/ros2_ws/src/user_project/launch/project_launch.py"
        
        entrypoint_content = f"""#!/bin/bash
set -e

# Подключаем глобальное окружение
source /opt/ros/humble/setup.bash

# Подключаем локальное окружение, если проект был скомпилирован
if [ -f /app/ros2_ws/install/setup.bash ]; then 
    source /app/ros2_ws/install/setup.bash
fi

# Запуск проекта
echo "🚀 Starting ROS 2 Blueprint Node..."
exec ros2 launch {launch_path}
"""
        with open(os.path.join(export_dir, "entrypoint.sh"), "w", newline='\n') as f:
            f.write(entrypoint_content)

        # 3. Генерируем Dockerfile с учетом лучших практик ROS 2
        dockerfile = """FROM osrf/ros:humble-desktop

# Устанавливаем базовые утилиты
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    ros-humble-turtlesim \\
    python3-rosdep \\
    && rm -rf /var/lib/apt/lists/*

# Инициализируем rosdep для подтягивания системных зависимостей
RUN sudo rosdep init || true
RUN rosdep update

WORKDIR /app/ros2_ws

# Копируем только исходники
COPY src /app/ros2_ws/src

# Автоматически ставим все зависимости из package.xml (те самые, которые мы спарсили!)
RUN apt-get update && rosdep install -y --from-paths src --ignore-src --rosdistro humble && rm -rf /var/lib/apt/lists/*

"""
        if has_cmake:
            dockerfile += """# === BUILD C++ ===
# colcon build - оркестратор сборки
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
"""
        else:
            dockerfile += "# === PYTHON MODE ===\n"

        dockerfile += """
# Настраиваем точку входа
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
"""
        with open(os.path.join(export_dir, "Dockerfile"), "w") as f:
            f.write(dockerfile)

        # 4. Генерируем README
        readme = """# ROS 2 Portable Project

1. Install Docker Desktop.
2. Open terminal in this folder.
3. Build the image:
   docker build -t my_ros2_project .
4. Run the container:
   docker run -it --net=host my_ros2_project
"""
        with open(os.path.join(export_dir, "README.txt"), "w") as f:
            f.write(readme)

        # 5. Архивация
        zip_path = os.path.join(project_path, "docker_package.zip")
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, dirs, files in os.walk(export_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, export_dir)
                    zipf.write(file_path, arcname)

        shutil.rmtree(export_dir)
        return zip_path