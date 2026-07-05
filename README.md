> **⚠️ Alpha Version / PoC:** This tool was created to simplify ROS 2 usage on Windows. Code quality is subject to improvement. Feedback is welcome!

# 🤖 ROS 2 Blueprint Studio

**Visual Scripting IDE for ROS 2 (Robot Operating System)**

ROS 2 Blueprint Studio is a standalone desktop application that allows you to create, configure, and run ROS 2 nodes using a visual node-based editor similar to Unreal Engine Blueprints.

It runs entirely on **Windows** (and Linux) without requiring a local ROS 2 installation, thanks to **Docker** integration.

<img width="1400" height="902" alt="image" src="https://github.com/user-attachments/assets/4e427ea7-a8b4-4dbc-b70e-c98e6cdb00fc" />

## 🚀 Key Features (v0.5.0)

🔌 **Visual Scripting & Custom Node Palette:** Design ROS 2 systems intuitively by connecting nodes. Save your own C++ or Python node templates into a customizable Node Library and drag-and-drop them onto the canvas for instant reuse.

📦 **Hierarchical Subgraphs (Macros):** Group related logic into composite nodes and "dive deep" into subsystems. Enter/exit subgraphs, dissolve groups (Del) or delete them with contents (Ctrl+Del) — with full save/load persistence.

🐳 **Deploy Groups → docker-compose:** Each subgraph can become its own deployment unit. The studio generates per-group launch files and a ready-to-use `docker-compose.yml`, so one visual graph maps onto multiple isolated containers.

🐍 & ⚙️ **C++ / Python — both alive end-to-end:** Full workflow for both languages: graph → generated code → build (colcon for C++) → live nodes in Docker. Standard `CMakeLists.txt` / `package.xml` are generated automatically with Smart Dependency Scan.

📝 **Real Code Editing with Live Sync:** Double-click a node to open its actual source file in your system editor. External edits are watched and synced back into the graph — and manual changes survive regeneration.

🦊 **Foxglove Visualization — no X server required:** One click installs and starts `foxglove_bridge` inside the container and opens Foxglove for live 3D visualization (markers, topics, cameras). No more XLaunch pain on Windows. Classic X11/RViz mode is still available as an option.

💾 **Robust Project System & UI Persistence:** Blueprints and node properties are saved as structured YAML. Node coordinates, dynamic ports, colors, and workspace layout are preserved between sessions.

📤 **Portable Docker Export:** Export your whole system as a self-contained Docker package (Dockerfile + sources + launch) that builds and runs anywhere — with automatic cleanup of stale files.

🚀 **Standalone Cross-Platform GUI:** Dark-themed Material Design interface built with Qt, dockable panels, and real-time terminal monitoring. Runs on Windows and Linux.

## 🛠️ Prerequisites

To run this application, you must have Docker installed and running.

* **Windows:** Install [Docker Desktop](https://www.docker.com/products/docker-desktop/).
* **Linux:** Install Docker Engine

## 📥 How to Run

### Option 1: Download Release (Recommended)

1.  Go to the **[Releases](../../releases)** page of this repository.
2.  Download the latest executable (`ROS2Studio.exe` for Windows).
3.  Run the file.
    > **Note:** The first launch might take a few minutes as it pulls the ROS 2 Docker image (~2GB).

### Option 2: Run from Source (Python)

If you want to modify the code or contribute:

1.  **Clone this repository:**
    ```bash
    git clone https://github.com/NeiroEvgen/ros2-blueprint-studio.git
    cd ros2-blueprint-studio
    ```

2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

3.  **Run the application:**
    ```bash
    python main.py
    ```

## ⚡ Quick Start Guide

1.  **Launch the App:** Ensure Docker is running in the background.
2.  **Add Nodes:** On the left "Library" panel, double-click on ` Publisher` and ` Subscriber`.
3.  **Connect:** Drag a wire from the **out** port of the Publisher to the **in** port of the Subscriber.
4.  **Run:** Click the **Run button** (Play icon) in the toolbar.
5.  **Observe:** Watch the "ROS 2 Output" panel at the bottom. You should see messages flowing between nodes.
6.  **Custom Logic:** Add a `Custom Code` node, double-click it, and write your own data processing logic!

## 🏗️ Building into EXE

If you want to build the executable yourself from the source:

### 🟦 Windows Build

Run this command in PowerShell or CMD:

```bash
pyinstaller --noconsole --onefile --clean --name="ROS2Studio" --add-data "assets;assets" --collect-all qt_material --hidden-import PySide6.QtSvg --hidden-import PySide6.QtXml --hidden-import NodeGraphQt main.py
```

### 🐧 Linux / macOS Build
```bash
pyinstaller --noconsole --onefile --clean --name="ROS2Studio" --add-data "assets:assets" --collect-all qt_material --hidden-import PySide6.QtSvg --hidden-import PySide6.QtXml --hidden-import NodeGraphQt main.py
```

