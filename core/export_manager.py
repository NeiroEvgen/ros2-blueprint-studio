import os
import shutil
import zipfile

class ExportManager:
    @staticmethod
    def export_to_portable_package(project_path, include_gui=False):
        """
        Создает архив с готовым к запуску Docker-контейнером.
        """
        export_dir = os.path.join(project_path, "export_docker")
        if os.path.exists(export_dir):
            shutil.rmtree(export_dir)
        os.makedirs(export_dir)

        src_dir = os.path.join(project_path, "src")
        target_src = os.path.join(export_dir, "src")
        
        # 1. Копируем исходники проекта
        shutil.copytree(src_dir, target_src)

        # 2. Определяем тип проекта
        has_cmake = os.path.exists(os.path.join(src_dir, "CMakeLists.txt"))
        
        # 3. Генерируем скрипт запуска (Entrypoint)
        entrypoint_content = "#!/bin/bash\n"
        entrypoint_content += "source /opt/ros/humble/setup.bash\n"
        
        if has_cmake:
            # Подключаем скомпилированное рабочее пространство
            entrypoint_content += "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi\n"
        
        # ВАЖНО: Указываем точный путь к launch файлу внутри контейнера
        launch_path = "/app/ros2_ws/src/user_project/launch/project_launch.py"
        entrypoint_content += f"exec ros2 launch {launch_path}\n"

        with open(os.path.join(export_dir, "entrypoint.sh"), "w", newline='\n') as f:
            f.write(entrypoint_content)

        # 4. Генерируем Dockerfile
        dockerfile = "FROM osrf/ros:humble-desktop\n\n"
        dockerfile += "WORKDIR /app/ros2_ws\n\n"
        
        dockerfile += "RUN apt-get update && apt-get install -y python3-pip ros-humble-turtlesim && rm -rf /var/lib/apt/lists/*\n\n"

        # Копируем исходники в контейнер (в папку user_project)
        dockerfile += "COPY src /app/ros2_ws/src/user_project\n\n"
        dockerfile += "COPY entrypoint.sh /entrypoint.sh\n"
        dockerfile += "RUN chmod +x /entrypoint.sh\n\n"

        if has_cmake:
            dockerfile += "# === BUILD C++ ===\n"
            dockerfile += "RUN /bin/bash -c '. /opt/ros/humble/setup.sh && colcon build --packages-select cpp_blueprints_pkg'\n\n"
        else:
            dockerfile += "# === PYTHON MODE ===\n\n"

        dockerfile += "ENTRYPOINT [\"/entrypoint.sh\"]\n"

        with open(os.path.join(export_dir, "Dockerfile"), "w") as f:
            f.write(dockerfile)

        # 5. README
        readme = """# How to run this robot
1. Install Docker Desktop.
2. Open terminal in this folder.
3. Build image:
   docker build -t my_robot_app .
4. Run it:
   docker run -it --net=host my_robot_app
"""
        with open(os.path.join(export_dir, "README.txt"), "w") as f:
            f.write(readme)

        # 6. Архивация
        zip_path = os.path.join(project_path, "docker_package.zip")
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, dirs, files in os.walk(export_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, export_dir)
                    zipf.write(file_path, arcname)

        # Удаляем временную папку
        shutil.rmtree(export_dir)
        return zip_path