# Hand Teleoperation Demo

Wave at your webcam — the 6-axis arm mirrors your hand in real time.

## Pipeline
webcam (host) → MediaPipe hand tracking → UDP :9877 →
HandUdpReceiverNode → /hand/pose → ArmControllerNode (smoothing) →
/arm/joint_cmd → ArmRenderNode → /arm_viz_markers → Foxglove 3D

## Setup (once)
1. Drag all 3 nodes from this palette onto the canvas (C++ project).
2. Host side needs Python 3.11 + two packages:
   pip install mediapipe==0.10.14 opencv-python
3. Get the host tracker script from the repo:
   github.com/NeiroEvgen/ros2-blueprint-studio → examples/hand_teleop/host/
   (keep it in an ASCII-only path — MediaPipe dislikes non-latin folders)

## Run
1. Press RUN and wait for the build to finish.
2. On the host:  python host_hand_tracker.py
   (a camera window with a hand skeleton should appear)
3. Press VISUALIZE → in Foxglove add a 3D panel,
   enable topic /arm_viz_markers, set Fixed frame = map.
4. Wave. It waves back.

## If the arm doesn't move
- Check UDP port: container must expose 9877/udp
  (docker ps should show 9877->9877/udp; recreate the session if not)
- Check data flow:  ros2 topic echo /hand/pose  (inside the container)
- No hand skeleton in the camera window = tracking problem, not ROS

## Tuning (ArmControllerNode)
- smoothing: alpha 0.25 (lower = smoother, laggier)
- yaw range: (x - 0.5) * 1.6
- hand lost for >0.5 s → arm drifts to idle pose