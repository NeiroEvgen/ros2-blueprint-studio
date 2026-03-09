> **⚠️ Alpha Version / PoC:** This tool was created to simplify ROS 2 usage on Windows. Code quality is subject to improvement. Feedback is welcome!

# 🤖 ROS 2 Blueprint Studio

**Visual Scripting IDE for ROS 2 (Robot Operating System)**

ROS 2 Blueprint Studio is a standalone desktop application that allows you to create, configure, and run ROS 2 nodes using a visual node-based editor similar to Unreal Engine Blueprints.

It runs entirely on **Windows** (and Linux) without requiring a local ROS 2 installation, thanks to **Docker** integration.

<img width="1400" height="902" alt="image" src="https://github.com/user-attachments/assets/4e427ea7-a8b4-4dbc-b70e-c98e6cdb00fc" />
  
## 🚀 Key Features (v0.5.0 Evolution)

* **🔌 Visual Scripting & Template Palette:** Design ROS 2 systems intuitively by connecting nodes. Version 0.5.0 introduces a C++ Template Library, allowing you to drag-and-drop native source templates directly onto the canvas for instant customization.

* **📦 Hierarchical Subgraphs (Macros):** Manage complexity by grouping related nodes into CompositeNodes. This recursive architecture allows you to "dive deep" into subsystems, keeping your top-level logic clean and readable even for 100+ node projects.

* **🐳 Docker & 🛠️ Bare-Metal Hybrid:** Run your nodes in isolated Docker containers (osrf/ros:humble-desktop) or use the new Native Dependency Generator to install and run everything directly on your host Ubuntu system.

* **🐍 & ⚙️ Native C++ / Python Workflow:** Full support for both languages with built-in code editors. C++ nodes are compiled natively within the environment, ensuring maximum performance for real-time R&D.

* **💾 Robust Project System & UI Persistence:** Blueprints are saved as structured JSON files. The new UI State manager ensures your node coordinates, colors, and workspace layout are perfectly preserved between sessions.

* **📝 One-Click Export & Launch:** Automatically generate standard ROS 2 packages, including CMakeLists.txt and package.xml. The Launch Generator creates Python launch files to deploy your entire system in one go.

* **🚀 Portable & Modern:** Runs as a single binary on Linux (or .exe on Windows) with zero installation required. Features a professional dark-themed Material Design interface with dockable panels for a personalized workspace.

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
    git clone [https://github.com/NeiroEvgen/ros2-visual-studio.git](https://github.com/NeiroEvgen/ros2-visual-studio.git)
    cd ros2-visual-studio
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
2.  **Add Nodes:** On the left "Library" panel, double-click on `Py: Publisher` and `Py: Subscriber`.
3.  **Connect:** Drag a wire from the **out** port of the Publisher to the **in** port of the Subscriber.
4.  **Run:** Click the **Run button** (Play icon) in the toolbar.
5.  **Observe:** Watch the "ROS 2 Output" panel at the bottom. You should see messages flowing between nodes.
6.  **Custom Logic:** Add a `Py: Custom Code` node, double-click it, and write your own data processing logic!

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

