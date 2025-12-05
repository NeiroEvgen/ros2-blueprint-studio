ROS 2 Blueprint Studio 

Visual Scripting IDE for ROS 2 (Robot Operating System)

ROS 2 Blueprint Studio is a standalone desktop application that allows you to create, configure, and run ROS 2 nodes using a visual node-based editor similar to Unreal Engine Blueprints.

It runs entirely on Windows (and Linux) without requiring a local ROS 2 installation, thanks to Docker integration.
<img width="1399" height="931" alt="image" src="https://github.com/user-attachments/assets/c27e1f6e-4da8-4d37-8a0c-54e057f956f6" />

Key Features

 Visual Scripting: Connect nodes with wires to create topics and data flows.

 Python &  C++ Support: Two separate tabs for creating graphs in Python or C++. The IDE automatically compiles C++ nodes inside the container.

 Docker Powered: No need to install ROS 2 locally. All nodes run inside an isolated Docker container (osrf/ros:humble-desktop).

 Custom Code Nodes: Double-click a node to open the built-in code editor and write your own logic.

 Subgraphs (Grouping): Group multiple nodes into a single block to keep your graph clean.

 Project System: Save and load your blueprints as JSON files.

 Modern UI: Dark theme, dockable panels, and user-friendly interface.
	

Alpha Version / PoC: This tool was created to simplify ROS2 usage on Windows. Code quality is subject to improvement. Feedback is welcome!
 

	Prerequisites

To run this application, you must have Docker installed and running.

Windows: Install Docker Desktop.

Linux: Install Docker Engine.

 How to Run

Option 1: Download EXE (Windows)

Go to the Docker Desktop page.

Download the latest Ros2Studio.exe.

Run the file. Note: The first launch might take a few minutes as it pulls the ROS 2 Docker image.

Option 2: Run from Source (Python)

If you want to modify the code:

Clone this repository:

git clone [https://github.com/YOUR_USERNAME/ros2-visual-studio.git](https://github.com/YOUR_USERNAME/ros2-visual-studio.git)
cd ros2-visual-studio


Install dependencies:

pip install -r requirements.txt


Run the application:

python main.py


 Quick Start Guide

Launch the App: Ensure Docker is running in the background.

Add Nodes: On the left "Library" panel, double-click on "Py: Publisher" and "Py: Subscriber".

Connect: Drag a wire from the out port of the Publisher to the in port of the Subscriber.

Run: Click the Run button (Play icon) in the toolbar.

Observe: Watch the "ROS 2 Output" panel at the bottom. You should see messages flowing between nodes.

Custom Logic: Add a "Py: Custom Code" node, double-click it, and write your own data processing logic!

 Building into EXE

If you want to build the executable yourself:

pip install pyinstaller
pyinstaller --noconsole --onefile --name="Ros2Studio" --hidden-import=NodeGraphQt --hidden-import=docker --hidden-import=PySide6.QtSvg --hidden-import=PySide6.QtXml --paths=. main.py


 Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

 License

This project is licensed under the MIT License.
