> **⚠️ Alpha Version / PoC:** This tool was created to simplify ROS 2 usage on Windows. Code quality is subject to improvement. Feedback is welcome!

# 🤖 ROS 2 Blueprint Studio

**Visual Scripting IDE for ROS 2 (Robot Operating System)**

ROS 2 Blueprint Studio is a standalone desktop application that allows you to create, configure, and run ROS 2 nodes using a visual node-based editor similar to Unreal Engine Blueprints.

It runs entirely on **Windows** (and Linux) without requiring a local ROS 2 installation, thanks to **Docker** integration.

<img width="1399" height="931" alt="image" src="https://github.com/user-attachments/assets/c27e1f6e-4da8-4d37-8a0c-54e057f956f6" />

## 🚀 Key Features

* **🔌 Visual Scripting:** Connect nodes with wires to create topics and data flows intuitively.
* **🐍 & ⚙️ Python & C++ Support:** Two separate tabs for creating graphs in Python or C++. The IDE automatically compiles C++ nodes inside the container.
* **🐳 Docker Powered:** No need to install ROS 2 locally. All nodes run inside an isolated Docker container (`osrf/ros:humble-desktop`).
* **📝 Custom Code Nodes:** Double-click a node to open the built-in code editor and write your own logic on the fly.
* **📦 Subgraphs (Grouping):** Group multiple nodes into a single block to keep your graph clean.
* **💾 Project System:** Save and load your blueprints as JSON files.
* **🎨 Modern UI:** Dark theme, dockable panels, and a user-friendly interface.

## 🛠️ Prerequisites

To run this application, you must have Docker installed and running.

* **Windows:** Install [Docker Desktop](https://www.docker.com/products/docker-desktop/).
* **Linux:** Install Docker Engine.

## 📥 How to Run

### Option 1: Download EXE (Windows) - *Recommended*

1.  Go to the **[Releases](../../releases)** page of this repository.
2.  Download the latest `Ros2Studio.exe`.
3.  Run the file.
    > **Note:** The first launch might take a few minutes as it pulls the ROS 2 Docker image (~2GB).

### Option 2: Run from Source (Python)

If you want to modify the code or contribute:

1.  **Clone this repository:**
    ```bash
    git clone https://github.com/NeiroEvgen/ros2-visual-studio.git
    cd ros2-visual-studio
    ```

2.  **Install dependencies:**
    ```bash
    cd RosStudio
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

```bash
pip install pyinstaller

pyinstaller --noconsole --onefile --name="Ros2Studio" --hidden-import=NodeGraphQt --hidden-import=docker --hidden-import=PySide6.QtSvg --hidden-import=PySide6.QtXml --paths=. main.py
