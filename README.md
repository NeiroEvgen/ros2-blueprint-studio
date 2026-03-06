> **⚠️ Alpha Version / PoC:** This tool was created to simplify ROS 2 usage on Windows. Code quality is subject to improvement. Feedback is welcome!

# 🤖 ROS 2 Blueprint Studio

**Visual Scripting IDE for ROS 2 (Robot Operating System)**

ROS 2 Blueprint Studio is a standalone desktop application that allows you to create, configure, and run ROS 2 nodes using a visual node-based editor similar to Unreal Engine Blueprints.

It runs entirely on **Windows** (and Linux) without requiring a local ROS 2 installation, thanks to **Docker** integration.

<img width="1400" height="902" alt="image" src="https://github.com/user-attachments/assets/4e427ea7-a8b4-4dbc-b70e-c98e6cdb00fc" />


## 🚀 Key Features

* **🔌 Visual Scripting:** Connect nodes with wires to create topics and data flows intuitively.
* **🚀 Portable Mode:** Runs as a single `.exe` file on Windows (or binary on Linux). No installation required.
* **🐳 Docker Powered:** No need to install ROS 2 locally. All nodes run inside an isolated Docker container (`osrf/ros:humble-desktop`).
* **🐍 & ⚙️ Python & C++ Support:** Two separate tabs for creating graphs in Python or C++. The IDE automatically compiles C++ nodes inside the container.
* **📝 Custom Code Nodes:** Double-click a node to open the built-in code editor and write your own logic on the fly.
* **📦 Subgraphs (Grouping):** Group multiple nodes into a single block to keep your graph clean.
* **💾 Project System:** Save and load your blueprints as JSON files.
* **🎨 Modern UI:** Professional dark theme (Material Design), dockable panels, and a user-friendly interface.

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
