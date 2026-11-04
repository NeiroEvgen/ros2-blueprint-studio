# Known limitations (0.6.0)

ROS 2 Blueprint Studio is alpha software. This page lists what is known to be missing or
different from what you might expect, so you do not have to find out the hard way.

## Platforms and runtime

- **Docker is required.** Nodes are built and run inside a container; there is no
  native-ROS mode.
- **ROS 2 Humble only.** The distribution is fixed in the generated build and rosdep steps.
- **Windows and Linux are the supported platforms.** macOS has not been tested; display
  forwarding for classic X11/RViz mode is not set up there. Foxglove mode needs no X server.
- **Most automated tests fake Docker.** The parameter bridge, lifecycle, launch with
  `params.yaml` and the service nodes are also tested against a real ROS 2 Humble container when
  the image is present; the rest of the container behaviour is covered by fakes and manual runs.
- **The packed EXE** reads nothing from the source tree: build it with the `--add-data` list from
  the README, or translations and the parameter bridge are missing. Its user palettes live in
  `~/.ros_blueprint_studio/user_palettes`.

## Build and run speed

- The **first Run** of a project creates a container and installs dependencies (apt + rosdep):
  expect roughly 30–60 seconds. Later Runs reuse the stopped container, skip the dependency
  step while `package.xml` and the extra apt list are unchanged, and take about 1 s (Python)
  to a few seconds (C++).
- The reused container keeps `build/` and `install/`. Results of nodes you deleted can linger
  there. There is no "recreate session" button yet; the container is recreated automatically
  when the ROS network settings change (domain, DDS, Real robot).
- Each Run appends phase timings to `<project>/.blueprint/timings.jsonl`.

## Graph and generated code

- **Names connect nodes, wires keep them in step.** ROS matches topic and service names; the
  generated code takes them from the node fields. Connecting a wire copies the name (publisher →
  subscriber, server → client) and Save warns about wired nodes whose names differ. Once you edit
  a node's code, the code decides. Exec-trigger ports (`ExecIn`/`ExecOut`) are not generated yet.
- **C++ node names come from the file.** The generated launch file names each C++ node after its
  file (the title on the canvas); the *Name* field is used by Python nodes only.
- **Run starts `project_launch.py` only.** Your own launch files in `src/launch` are kept, but
  you start them from a terminal in the session container.
- **Dependencies are detected from `#include` lines** against a built-in registry. An include
  that is not in the registry is reported as a warning, not installed.
- **Editing code in two places at once.** If a node's code is changed both in Studio and in
  the file on disk between two saves, the file wins and a warning is written to the log.
- Python nodes are started as plain scripts by the generated launch file; C++ nodes are built
  with colcon.

## Single canvas (default in 0.6.0)

- One canvas holds Python and C++ nodes together. The project is saved in the **v4** state
  format. Studio 0.5.x cannot open it; on the first v4 write the old state is kept as
  `.blueprint/state.v3.6.yaml.bak`.
- The old two-tab mode is still available (checkbox in the launcher, applies on next start).
  Opening a v4 project in that mode splits nodes by language; wires between a Python and a C++
  node are lost.

## Robots and remote hosts

- The core only provides a **generic SSH connection** (host, user, password). Vendor-specific
  robot integrations are not part of the core and are expected as extensions.
- **A local Run is isolated from the network** (`ROS_LOCALHOST_ONLY=1`); turn on Real robot to
  reach robots on the network. Deploy groups started with `docker-compose.yml` are not isolated:
  they are meant to talk across machines.
- Real-robot networking (DDS settings, multi-host discovery) is configurable per deploy group
  but has not been validated against physical hardware by the project.
- A DDS QoS profile set on the Containers tab is mounted for `docker-compose` services, not for
  the local Run.
- The service nodes use `example_interfaces`, which is in the default image
  (`osrf/ros:humble-desktop`) but not in the smaller `ros:humble-ros-core/ros-base` images.

## Interface

- UI languages: English and Russian. Switching the language updates the toolbar immediately;
  other panels pick it up after a restart.
- Deploy groups: a node's explicit group wins, then its subgraph's, otherwise the outermost
  subgraph name, otherwise `main`.
