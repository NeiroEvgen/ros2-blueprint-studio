import os
import json
import shutil
import platform

class WorkspaceManager:
    # 1. Получаем путь к домашней директории пользователя
    USER_HOME = os.path.expanduser("~")
    
    # 2. Строим путь к Документам
    DOCUMENTS_DIR = os.path.join(USER_HOME, "Documents")
    
    # 3. Целевая папка для проектов
    WORKSPACE_DIR = os.path.join(DOCUMENTS_DIR, "BlueprintStudioProjects")

    @staticmethod
    def get_workspace_root():
        """Возвращает путь к папке с проектами, создавая её при необходимости"""
        if not os.path.exists(WorkspaceManager.WORKSPACE_DIR):
            try:
                os.makedirs(WorkspaceManager.WORKSPACE_DIR)
                print(f"📁 Created new workspace at: {WorkspaceManager.WORKSPACE_DIR}")
            except OSError as e:
                print(f"❌ Error creating workspace: {e}")
                # Fallback на случай проблем с правами - создаем рядом со скриптом
                fallback = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "BlueprintStudioProjects")
                fallback = os.path.abspath(fallback)
                if not os.path.exists(fallback):
                    os.makedirs(fallback)
                return fallback
                
        return WorkspaceManager.WORKSPACE_DIR

    @staticmethod
    def list_projects():
        root = WorkspaceManager.get_workspace_root()
        if not os.path.exists(root):
            return []
        # Возвращаем только папки
        return [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]

    @staticmethod
    def create_project(project_name, project_type="python"):
        """
        Создает новый проект с полной структурой ROS 2 пакета.
        project_type: 'python' или 'cpp'
        """
        root = WorkspaceManager.get_workspace_root()
        path = os.path.join(root, project_name)
        
        if os.path.exists(path):
            raise FileExistsError(f"Project '{project_name}' already exists!")
            
        # 1. Создаем структуру папок
        os.makedirs(path)
        os.makedirs(os.path.join(path, "src"))
        os.makedirs(os.path.join(path, "launch"))
        os.makedirs(os.path.join(path, "docker"))
        os.makedirs(os.path.join(path, ".blueprint")) # Для метаданных редактора

        # 2. Сохраняем конфиг Blueprint
        config = {
            "name": project_name,
            "type": project_type,
            "version": "1.0.0"
        }
        
        with open(os.path.join(path, "project_config.json"), 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=4)

        # 3. Генерируем ROS-файлы (package.xml, CMakeLists.txt)
        WorkspaceManager._create_package_xml(path, project_name)
        WorkspaceManager._create_cmakelists(path, project_name)
        WorkspaceManager._create_docker_template(path, project_name)

        return path

    @staticmethod
    def get_project_type(project_path):
        """
        Читает тип проекта из конфига или угадывает по файлам.
        """
        config_path = os.path.join(project_path, "project_config.json")
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    data = json.load(f)
                    return data.get("type", "python")
            except:
                pass
        
        # Для старых проектов (без конфига) - угадываем
        src_dir = os.path.join(project_path, "src")
        if os.path.exists(src_dir):
            for f in os.listdir(src_dir):
                if f.endswith('.cpp'):
                    return "cpp"
        
        return "python"

    # --- ГЕНЕРАТОРЫ ФАЙЛОВ ---

    @staticmethod
    def _create_package_xml(path, name):
        content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>Created with ROS2 Blueprint Studio</description>
  <maintainer email="user@todo.todo">User</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
        with open(os.path.join(path, "package.xml"), "w", encoding='utf-8') as f:
            f.write(content)

    @staticmethod
    def _create_cmakelists(path, name):
        # Гибридный CMake для поддержки и C++, и Python скриптов в одной папке src
        content = f"""cmake_minimum_required(VERSION 3.8)
project({name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# === PYTHON SCRIPTS INSTALLATION ===
install(DIRECTORY src/
  DESTINATION lib/${{PROJECT_NAME}}
  FILES_MATCHING PATTERN "*.py"
)

# === LAUNCH FILES INSTALLATION ===
install(DIRECTORY launch
  DESTINATION share/${{PROJECT_NAME}}
)

# === C++ NODES AUTO-INJECTION MARKER ===
# [AUTO-GEN-CPP-NODES]

ament_package()
"""
        with open(os.path.join(path, "CMakeLists.txt"), "w", encoding='utf-8') as f:
            f.write(content)

    @staticmethod
    def _create_docker_template(path, name):
        # Базовый композ для Docker
        content = f"""version: '3'
services:
  {name}_node:
    image: osrf/ros:humble-desktop
    volumes:
      - ./:/ros2_ws/src/{name}
    working_dir: /ros2_ws
    command: bash -c "colcon build && source install/setup.bash && ros2 launch {name} deployment.launch.py"
    network_mode: host
    privileged: true
"""
        with open(os.path.join(path, "docker", "docker-compose.yaml"), "w", encoding='utf-8') as f:
            f.write(content)