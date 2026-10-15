# Known limitations (0.6.0)

ROS 2 Blueprint Studio is alpha software. This page lists what is known to be missing or
different from what you might expect, so you do not have to find out the hard way.

## Platforms and runtime

- **Docker is required.** Nodes are built and run inside a container; there is no
  native-ROS mode.
- **ROS 2 Humble only.** The distribution is fixed in the generated build and rosdep steps.
- **Windows and Linux are the supported platforms.** macOS has not been tested; display
  forwarding for classic X11/RViz mode is not set up there. Foxglove mode needs no X server.
- **Automated tests do not start a real Docker or ROS 2.** Container behaviour is covered by
  fakes; end-to-end runs are verified manually. Report anything that differs on your machine.

## Build and run speed

- The **first Run** of a project creates a container and installs dependencies (apt + rosdep):
  expect roughly 30–60 seconds. Later Runs reuse the stopped container, skip the dependency
  step while `package.xml` and the extra apt list are unchanged, and take about 1 s (Python)
  to a few seconds (C++).
- The reused container keeps `build/` and `install/`. Results of nodes you deleted can linger
  there. There is no "recreate session" button yet.
- Each Run appends phase timings to `<project>/.blueprint/timings.jsonl`.

## Graph and generated code

- **Wires are not code.** Generated nodes take topic names from node properties; wires
  document the data flow and are saved with the project. Exec-trigger ports (`ExecIn`/`ExecOut`)
  are not generated yet.
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
- Real-robot networking (DDS settings, multi-host discovery) is configurable per deploy group
  but has not been validated against physical hardware by the project.

## Interface

- UI languages: English and Russian. Switching the language updates the toolbar immediately;
  other panels pick it up after a restart.
- Deploy groups: a node's explicit group wins, then its subgraph's, otherwise the outermost
  subgraph name, otherwise `main`.
