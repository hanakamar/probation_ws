# Probation Task: Going Through Gate with Unity Simulation
Applicant: Hana Kamarudeen
Video: [https://drive.google.com/file/d/1Z53qzfEDD625G4rnZ3P15ugKspY23H5Q/view?usp=sharing](url)

## Build & Run 
**1. Navigate to your workspace:**
 ```bash
cd ~/probation_ws
colcon build --symlink-install
source install/setup.bash
   ```
**2. Start the ROS–Unity TCP bridge:**
 ```bash
ros2 run ros_tcp_endpoint default_server_endpoint
   ```
**3. Launch the Unity simulation.**

**4. In a new terminal, run the AUV gate navigation node:**
 ```bash
ros2 run auv_gate_nav test_auv_motion
  ```
The node will:
- Arm the vehicle and set it to GUIDED mode.
- Subscribe to bounding box detections from the Unity camera.
- Align horizontally and vertically with the detected gate.
- Move forward through the gate once aligned.
  
## Notes
- Requires vision_msgs, geometry_msgs, std_msgs, and mavros_msgs packages.
- Make sure Unity publishes detections on /main_camera/detection/bounding_boxes.
- The code assumes bounding boxes contain a "gate" label for detection.
