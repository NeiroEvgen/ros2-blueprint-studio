import os
import shutil
import zipfile
import re

class ExportManager:
    @staticmethod
    def export_to_portable_package(project_path, include_gui=False):
        """
        Creates an archive with a ready-to-run Docker container.
        Follows a (logically) multi-stage layout for building the ROS 2 workspace.
        Parses the code and adds custom dependencies (includes) automatically.
        """
        export_dir = os.path.join(project_path, "export_docker")
        if os.path.exists(export_dir):
            shutil.rmtree(export_dir)
        os.makedirs(export_dir)

        # 1. Proper workspace structure
        src_dir = os.path.join(project_path, "src")
        target_src = os.path.join(export_dir, "src", "user_project") 
        
        # Copy the sources into a subfolder so colcon sees the package correctly
        shutil.copytree(src_dir, target_src,
                ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))

        deps = set()
        
        # Message packages: #include "nav_msgs/msg/odometry.hpp" -> nav_msgs
        pattern_msg = re.compile(r'#include[ \t]+["<]([A-Za-z_][A-Za-z0-9_]*)/(msg|srv|action)/[^">\n]+[">]')
        # Well-known ROS libraries (tf2_ros, cv_bridge, ...)
        pattern_libs = re.compile(r'#include\s+["<](tf2_ros|tf2|tf2_geometry_msgs|image_transport|cv_bridge|opencv2)/[^">]+[">]')
        
        # Scan the copied sources
        for root_dir, _, files in os.walk(target_src):
            for file in files:
                if file.endswith(('.cpp', '.hpp', '.h')):
                    with open(os.path.join(root_dir, file), 'r', encoding='utf-8') as f:
                        content = f.read()
                        
                        for match in pattern_msg.findall(content):
                            deps.add(match[0])
                        for match in pattern_libs.findall(content):
                            if match == "opencv2":
                                deps.add("opencv")
                            else:
                                deps.add(match)
        
        # rclcpp is usually listed already, keep it anyway
        deps.discard('rclcpp')

        # Found custom includes: inject them into the build files
        if deps:
            # A) Patch package.xml for rosdep (so Docker installs the packages via apt)
            package_xml_path = os.path.join(target_src, "package.xml")
            if os.path.exists(package_xml_path):
                with open(package_xml_path, 'r', encoding='utf-8') as f:
                    pkg_xml = f.read()
                
                for dep in deps:
                    # the package is not in the manifest yet: insert it before </package>
                    if f"<depend>{dep}</depend>" not in pkg_xml and f"<build_depend>{dep}</build_depend>" not in pkg_xml:
                        pkg_xml = pkg_xml.replace("</package>", f"  <depend>{dep}</depend>\n</package>")
                
                with open(package_xml_path, 'w', encoding='utf-8') as f:
                    f.write(pkg_xml)

            # B) Patch CMakeLists.txt for colcon
            cmake_path = os.path.join(target_src, "CMakeLists.txt")
            if os.path.exists(cmake_path):
                with open(cmake_path, 'r', encoding='utf-8') as f:
                    cmake_txt = f.read()
                
                find_pkgs_str = ""
                cmake_deps = []
                for dep in deps:
                    # add find_package if it is not there yet
                    actual_cmake_name = "OpenCV" if dep == "opencv" else dep
                    cmake_deps.append(actual_cmake_name)
                    
                    if f"find_package({dep}" not in cmake_txt:
                        find_pkgs_str += f"find_package({dep} REQUIRED)\n"
                
                if find_pkgs_str:
                    # insert the new packages after the base ament_cmake
                    if "find_package(ament_cmake REQUIRED)" in cmake_txt:
                        cmake_txt = cmake_txt.replace("find_package(ament_cmake REQUIRED)", f"find_package(ament_cmake REQUIRED)\n{find_pkgs_str}")
                    else:
                        cmake_txt = find_pkgs_str + "\n" + cmake_txt
                    
                    # Add these dependencies to every executable/node:
                    # find "ament_target_dependencies(NODE_NAME current_deps)" and append the new ones
                    extra_deps_str = " ".join(cmake_deps)
                    target_deps_pattern = re.compile(r'(ament_target_dependencies\s*\(\s*[^ \)]+)(.*?)\)')
                    cmake_txt = target_deps_pattern.sub(rf'\1\2 {extra_deps_str})', cmake_txt)

                with open(cmake_path, 'w', encoding='utf-8') as f:
                    f.write(cmake_txt)
        # =========================================================

        has_cmake = os.path.exists(os.path.join(target_src, "CMakeLists.txt"))
        
        # 2. Entrypoint
        launch_path = "/app/ros2_ws/src/user_project/launch/project_launch.py"
        
        entrypoint_content = f"""#!/bin/bash
set -e

# Global ROS environment
source /opt/ros/humble/setup.bash

# Local environment, if the project has been built
if [ -f /app/ros2_ws/install/setup.bash ]; then 
    source /app/ros2_ws/install/setup.bash
fi

# Run the project
echo " Starting ROS 2 Blueprint Node..."
exec ros2 launch {launch_path}
"""
        with open(os.path.join(export_dir, "entrypoint.sh"), "w", encoding='utf-8', newline='\n') as f:
            f.write(entrypoint_content)

        # 3. Dockerfile following ROS 2 practice
        dockerfile = """FROM osrf/ros:humble-desktop

# Base tools
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    ros-humble-turtlesim \\
    python3-rosdep \\
    && rm -rf /var/lib/apt/lists/*

# rosdep to pull system dependencies
RUN sudo rosdep init || true
RUN rosdep update

WORKDIR /app/ros2_ws

# Copy only the sources
COPY src /app/ros2_ws/src

# Install all dependencies from package.xml (the ones parsed above)
RUN apt-get update && rosdep install -y --from-paths src --ignore-src --rosdistro humble && rm -rf /var/lib/apt/lists/*

"""
        if has_cmake:
            dockerfile += """# === BUILD C++ ===
# colcon build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
"""
        else:
            dockerfile += "# === PYTHON MODE ===\n"

        dockerfile += """
# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
"""
        with open(os.path.join(export_dir, "Dockerfile"), "w", encoding='utf-8') as f:
            f.write(dockerfile)

        # 4. README
        readme = """# ROS 2 Portable Project

1. Install Docker Desktop.
2. Open terminal in this folder.
3. Build the image:
   docker build -t my_ros2_project .
4. Run the container:
   docker run -it --net=host my_ros2_project
"""
        with open(os.path.join(export_dir, "README.txt"), "w", encoding='utf-8') as f:
            f.write(readme)

        # 5. Archive
        zip_path = os.path.join(project_path, "docker_package.zip")
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, dirs, files in os.walk(export_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, export_dir)
                    zipf.write(file_path, arcname)

        shutil.rmtree(export_dir)
        return zip_path