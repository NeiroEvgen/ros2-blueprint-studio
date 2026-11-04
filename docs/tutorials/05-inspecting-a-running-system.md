# 5. Looking at a running system

Official tutorials: the whole [Beginner: CLI tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
section: understanding nodes, topics, services, parameters and actions, `rqt_console`, launching
nodes, recording and playing back data.

The official section teaches these tools on `turtlesim` before you write any code. Here they come
after chapters 2 and 4, on your own nodes: a publisher/subscriber pair and a service/client pair in
one project. Everything below is real output from that project.

## Two ways to look

- **Studio** shows the everyday things without a terminal: logs (ROS2 Dashboard), parameters
  (Parameters window, chapter 6), node states (Freeze/Unfreeze, chapter 8).
- **The `ros2` command line** shows everything else. The session container is a normal ROS 2
  Humble environment, so every command of the official tutorials works there:

  ```bash
  docker exec -it ros2_orchestrator_session bash
  source /opt/ros/humble/setup.bash
  ```

  This replaces the official *Configuring environment* step: ROS is installed and sourced in the
  container, you never install it on your machine. On a robot connected on the **Remote** tab, the
  terminal button opens an ssh session there.

## Nodes

```console
$ ros2 node list
/add_two_ints_client
/add_two_ints_server
/listener
/talker

$ ros2 node info /listener
/listener
  Subscribers:
    /chatter: std_msgs/msg/String
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /listener/describe_parameters: rcl_interfaces/srv/DescribeParameters
    ...
```

Every node also has `/parameter_events`, `/rosout` and six parameter services: ROS adds them,
they are not in your code.

## Topics

```console
$ ros2 topic list -t
/chatter [std_msgs/msg/String]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]

$ ros2 topic info /chatter
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1

$ ros2 topic echo /chatter --once
data: 'Py: 16'
---

$ ros2 topic hz /chatter
average rate: 2.000
	min: 0.499s max: 0.501s std dev: 0.00046s window: 4

$ ros2 interface show std_msgs/msg/String
string data
```

`topic info` is the quickest check when a subscriber stays silent: *Subscription count: 0* or a
different type means the names or types do not match (chapter 2). `topic hz` confirms the
publisher's *Interval*: 0.5 s, 2 Hz.

## Services

```console
$ ros2 service list -t
/add_two_ints [example_interfaces/srv/AddTwoInts]
...

$ ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 40, b: 2}"
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=40, b=2)

response:
example_interfaces.srv.AddTwoInts_Response(sum=42)
```

## Parameters

```console
$ ros2 param list /talker
  use_sim_time
$ ros2 param get /talker use_sim_time
Boolean value is: False
```

`ros2 param set` works too, but for tuning use Studio's **Parameters** window (right-click a node →
*Parameters…*): sliders for declared ranges, the node's own refusals shown, values saved into the
project. That is [chapter 6](06-parameters.md).

## Actions

`ros2 action list` prints nothing for this project: none of its nodes has an action. The palette
has no action nodes yet; write one in a custom node following the official
*Writing an action server and client* tutorial.

## Logs: instead of `rqt_console`

The **ROS2 Dashboard** tab is Studio's `rqt_console`: one row per process with its last message,
double-click a row for the whole log of that node. The **System Log** tab is Studio itself: build
output, saving, Docker, Remote.

`rqt` tools need a display. The session image has them, but they need an X server: Studio's
*X11 (XLaunch)* visualization mode (on Windows with XLaunch running). **Visualize** in the default *Foxglove*
mode needs no X server: topics, plots and 3D in the browser.

## Recording and playing back data

```console
$ ros2 bag record -o demo_bag /chatter
...
[INFO] [rosbag2_recorder]: Recording...
^C
$ ros2 bag info demo_bag
Files:             demo_bag_0.db3
Duration:          4.499911815s
Messages:          10
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 10 | Serialization Format: cdr
```

> [!WARNING]
> Stop the recording with **Ctrl+C**: the bag is closed properly only then. Killing the process
> leaves a bag without metadata that `ros2 bag info` cannot read.

```bash
ros2 bag play demo_bag                                  # back on /chatter: listener hears it too
ros2 bag play demo_bag --remap /chatter:=/chatter_replay   # or on another topic
```

The bag is written inside the container. To keep it, write it under the project:
`ros2 bag record -o /root/ros2_ws/src/user_project/bags/demo /chatter` lands in your project's
`src/bags/` on disk (and is synced to the robot with the rest of `src/`, so delete what you do not
need).

## Official tutorial → Studio

| Official (Beginner: CLI tools) | In Studio |
|---|---|
| Configuring environment | nothing to do: ROS 2 runs in the session container |
| turtlesim, `ros2`, rqt | your own nodes; `ros2` in the container; rqt in X11 mode, Foxglove otherwise |
| Understanding nodes / topics / services | `ros2 node`, `ros2 topic`, `ros2 service` in the container |
| Understanding parameters | Parameters window (chapter 6); `ros2 param` in the container |
| Understanding actions | `ros2 action` in the container; no action nodes in the palette yet |
| `rqt_console` | ROS2 Dashboard tab |
| Launching nodes | **Run** starts the generated launch file (chapter 7) |
| Recording and playing back data | `ros2 bag` in the container |
