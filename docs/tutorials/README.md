# ROS 2 tutorials in Blueprint Studio

The official [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html) teach ROS 2 from the
terminal: create a workspace, create a package, write a node, build with colcon, run it.
These chapters walk the same path **in Blueprint Studio**. For every step they show:

- what you do in Studio instead of the terminal;
- what Studio generates for you, and where to find it in the project;
- what still matters: the ROS 2 concepts Studio does not hide, and the places where Studio
  behaves differently from the official tutorial.

Read the official tutorial alongside: it explains the concepts, these chapters show the tool.

## Before you start

- Docker is installed and running (Docker Desktop on Windows). Studio runs ROS 2 Humble in a
  container, so you do not need a local ROS 2 installation.
- Studio is installed (see [Install](https://github.com/NeiroEvgen/ros2-blueprint-studio#install)). The first Run pulls the ROS 2 image
  (about 2 GB) and takes a few minutes.

## Chapters

| # | Studio chapter | Official tutorial | Status |
|---|---|---|---|
| 1 | [Workspace, package and build](01-workspace-and-package.md) | Creating a workspace · Creating a package · Using colcon to build packages | ready |
| 2 | [Publisher and subscriber (Python)](02-publisher-subscriber-python.md) | Writing a simple publisher and subscriber (Python) | ready |
| 3 | [Publisher and subscriber (C++)](03-publisher-subscriber-cpp.md) | Writing a simple publisher and subscriber (C++) | ready |
| 4 | [Service and client](04-service-client.md) | Writing a simple service and client (C++ / Python) | ready |
| 5 | [Looking at a running system](05-inspecting-a-running-system.md) | Beginner: CLI tools (nodes, topics, services, parameters, actions, rqt_console, bag) | ready |
| 6 | [Parameters, tuned live](06-parameters.md) | Using parameters in a class (C++ / Python) · Understanding parameters | ready |
| 7 | [Launch files](07-launch-files.md) | Launch: Creating a launch file · Integrating launch files into packages | ready |
| 8 | Managed (lifecycle) nodes | Demos: Managed nodes | planned |
| 9 | The same project on a real robot | (Studio only: Remote tab) | planned |

## How the examples were checked

> [!TIP]
> Every chapter's project was built in Studio and run in a real ROS 2 Humble container; the
> screenshots and logs come from those runs. If something looks different on your machine,
> that is a bug: please report it.
