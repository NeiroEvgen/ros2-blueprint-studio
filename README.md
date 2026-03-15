> **⚠️ Alpha Version / PoC:** This tool was created to simplify ROS 2 usage on Windows. Code quality is subject to improvement. Feedback is welcome!

# 🤖 ROS 2 Blueprint Studio

**Visual Scripting IDE for ROS 2 (Robot Operating System)**

ROS 2 Blueprint Studio is a standalone desktop application that allows you to create, configure, and run ROS 2 nodes using a visual node-based editor similar to Unreal Engine Blueprints.

It runs entirely on **Windows** (and Linux) without requiring a local ROS 2 installation, thanks to **Docker** integration.

<img width="1400" height="902" alt="image" src="https://github.com/user-attachments/assets/4e427ea7-a8b4-4dbc-b70e-c98e6cdb00fc" />
  
🚀 Key Features (v0.5.0 Evolution)

🔌 Visual Scripting & Custom Node Palette: Design ROS 2 systems intuitively by connecting nodes. Version 0.5.0 introduces a customizable Node Library, allowing you to save your own C++ or Python node templates and drag-and-drop them directly onto the canvas for instant reuse.

📦 Hierarchical Subgraphs (Macros): Manage complexity by grouping related logic into CompositeNodes. This nested architecture allows you to "dive deep" into subsystems, keeping your top-level architecture clean and readable even for large-scale projects.

🐳 Flexible Deployment (Docker & Bare-Metal): Run your entire generated architecture in isolated Docker containers (osrf/ros:humble-desktop) with a single click, or use the Native Dependency Generator to fetch requirements and run natively on your host Ubuntu system.

🐍 & ⚙️ Seamless C++ / Python Workflow: Full support for both languages with built-in code editors. The studio automatically generates standard CMakeLists.txt and package.xml files and builds your C++ nodes using standard colcon tools under the hood.
>🚧 Work in Progress: The Python compiler is currently taking a nap due to core refactoring. Stick to C++ for now!

💾 Robust Project System & UI Persistence: Blueprints and node properties are cleanly saved as structured YAML files. The UI State manager ensures your node coordinates, dynamic ports, colors, and workspace layout are perfectly preserved between sessions.

📝 One-Click Export & Launch: The studio acts as an intelligent boilerplate generator. It scans your code for dependencies (Smart Scan) and creates standard ROS 2 packages with Python launch files to deploy your entire system instantly.

🚀 Standalone Cross-Platform GUI: Features a professional dark-themed Material Design interface built with Qt. Designed to run as a standalone application on Linux and Windows, offering dockable panels and real-time terminal monitoring.

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

