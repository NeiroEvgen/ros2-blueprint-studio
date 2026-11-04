# 7. Launch files

Official tutorials: [Creating a launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) ·
[Integrating launch files into ROS 2 packages](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html) ·
the rest of the [Launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) section

## What the official tutorial does

A launch file starts several nodes at once, with their names, namespaces, remappings and
parameters, instead of one `ros2 run` per terminal. The tutorial writes a Python launch file with
`Node(package=..., executable=..., name=..., namespace=..., remappings=[...])` actions, installs it with
the package and starts it with `ros2 launch`.

## What Studio does for you

You have been using launch files since chapter 2: **Run** starts `src/launch/project_launch.py`,
which Studio regenerates on every Save from the nodes on the canvas.

```python
PARAMS_FILE = '/root/ros2_ws/src/user_project/config/params.yaml'

def generate_launch_description():
    ld = LaunchDescription()
    params = [PARAMS_FILE] if os.path.exists(PARAMS_FILE) else []

    # C++ nodes: from the built package
    ld.add_action(Node(package='my_project', executable='talker', name='talker',
                       parameters=params, output='screen', emulate_tty=True))
    # Python nodes: the script itself
    ld.add_action(ExecuteProcess(
        cmd=['python3', '-u', '/root/ros2_ws/src/user_project/python/listener.py']
            + (['--ros-args', '--params-file', PARAMS_FILE] if params else []),
        output='screen', shell=True, emulate_tty=True))
    return ld
```

- one action per node on the canvas: `Node` for C++, `ExecuteProcess` for Python scripts
  (chapter 1 explains why Python nodes are scripts);
- the saved parameters of chapter 6 are passed to every node;
- for C++ projects `CMakeLists.txt` installs the `launch/` folder with the package, the official
  *Integrating launch files into packages* step.

## Deploy groups: one launch file per group

Select nodes, right-click → **Assign to container...** and type a group name, say `robot` for the
talker and `pc` for the listener. On Save you get, next to `project_launch.py`:

```
src/launch/robot_launch.py   <- only the nodes of group "robot"
src/launch/pc_launch.py      <- only the nodes of group "pc"
docker-compose.yml           <- one service per group
```

`project_launch.py` still starts everything (that is what **Run** uses). A group file starts its
part only: `ros2 launch .../robot_launch.py` in the session container shows just `/talker` in
`ros2 node list`. `docker-compose.yml` runs each group in its own container (built with colcon
first when the group has C++), which is how one graph is split between a robot and a PC.
The group's container settings (image, DDS, network simulation) are on the **Containers** tab.

Compose services use host networking and are meant to talk across machines: unlike a local
**Run**, they are not isolated from the network.

## Your own launch file

Studio rewrites `project_launch.py` and the group files on every Save, so do not edit them.
Everything the generator does not know about (remappings, namespaces, arguments, conditions,
event handlers, other packages) goes into **your own file in `src/launch/`**. Studio never touches
files it did not generate.

`src/launch/my_launch.py`, which starts Studio's nodes plus a second listener under another name,
on another topic:

```python
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

HERE = os.path.dirname(os.path.realpath(__file__))
PYTHON = os.path.join(HERE, '..', 'python')


def generate_launch_description():
    return LaunchDescription([
        # everything on the canvas, exactly as Run starts it
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, 'project_launch.py'))),
        # one more listener: same script, another node name, another topic
        ExecuteProcess(
            cmd=['python3', '-u', os.path.join(PYTHON, 'listener.py'),
                 '--ros-args', '-r', '__node:=listener_copy', '-r', '/chatter:=/chatter_copy'],
            output='screen'),
    ])
```

Start it in the session container:

```bash
docker exec -it ros2_orchestrator_session bash
source /opt/ros/humble/setup.bash
ros2 launch /root/ros2_ws/src/user_project/launch/my_launch.py
```

```console
$ ros2 node list
/listener
/listener_copy
/talker
```

- **Include, do not copy.** `IncludeLaunchDescription` of `project_launch.py` keeps your file
  correct when you add nodes on the canvas.
- **Python nodes take ROS arguments on the command line.** The official tutorial renames and
  remaps with `Node(name=..., remappings=...)`, which needs a package executable. A Studio Python
  node is a script, so the same goes after `--ros-args`: `-r __node:=new_name`, `-r __ns:=/robot`,
  `-r /from:=/to`, `-p name:=value`. For C++ nodes `Node(package=..., executable=...)` works as in
  the official tutorial.

> [!NOTE]
> **Run does not start your file.** **Run** always starts `project_launch.py`. Start your own
> launch file from the container terminal as above (stop Studio's Run first, or you get the nodes
> twice).

## Where Studio differs

| Official tutorial | Studio |
|---|---|
| you write the launch file | `project_launch.py` is generated from the canvas on every Save |
| `Node(package, executable, name)` | C++ nodes: the same; Python nodes: `ExecuteProcess` of the script |
| `namespace=`, `remappings=` | your own launch file (`--ros-args -r` for Python nodes) |
| `parameters=[...]` | `src/config/params.yaml` from the Parameters window (chapter 6) |
| installing launch files with the package | done in the generated `CMakeLists.txt` (C++) |
| one launch file for several machines | deploy groups: `<group>_launch.py` + `docker-compose.yml` |
| substitutions, event handlers, large projects | your own launch file, exactly as in the official tutorials |
