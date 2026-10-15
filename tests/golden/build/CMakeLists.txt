# === BLUEPRINT AUTO BEGIN — regenerated on every Save, do not edit ===
cmake_minimum_required(VERSION 3.8)
project(my_proj_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_pkg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_executable(listener cpp/listener.cpp)
ament_target_dependencies(listener cv_bridge example_interfaces geometry_msgs my_pkg OpenCV rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_msgs tf2_ros trajectory_msgs)
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})

add_executable(talker cpp/talker.cpp)
ament_target_dependencies(talker cv_bridge example_interfaces geometry_msgs my_pkg OpenCV rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_msgs tf2_ros trajectory_msgs)
install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# === BLUEPRINT AUTO END ===

# === BLUEPRINT USER BEGIN — never touched by the generator ===
# Сюда пиши всё, что генератор не знает сам. Примеры:
#   target_sources(MyNode PRIVATE cpp/kinematics_lib/trajectory_solver.cpp)
#   find_package(yaml-cpp REQUIRED)
#   target_link_libraries(MyNode yaml-cpp)
# === BLUEPRINT USER END ===

ament_package()
