ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}

# === AUTO: dependencies from graph nodes (regenerated on save) ===
RUN apt-get update && apt-get install -y \
        ros-humble-rclcpp \
        ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*
# === END AUTO ===

# === MANUAL: user libraries (Library tab) ===
# === END MANUAL ===
