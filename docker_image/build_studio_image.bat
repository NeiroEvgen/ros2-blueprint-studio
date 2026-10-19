@echo off
REM Builds the Blueprint Studio base image: blueprint-studio:humble
REM Needs internet. First build takes ~10-20 minutes.
REM ASCII-only on purpose: cmd.exe mangles UTF-8 Cyrillic.
setlocal
set HERE=%~dp0
docker build -t blueprint-studio:humble -f "%HERE%Dockerfile.studio" "%HERE%"
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
echo.
echo Done. Checking baked packages:
docker run --rm blueprint-studio:humble bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E controller_manager\|moveit_ros_move_group\|foxglove_bridge\|turtlesim\|kdl_parser"
