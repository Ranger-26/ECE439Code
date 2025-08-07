# ros2_robot_arm

This ROS2 package implements stereo vision-based ball tracking and robot arm control using the following nodes and topics:

## Nodes and Topics

- **stereo_vision_position_detection**
  - Publishes: `/object_position` (Float32MultiArray)
  - Detects ball position using stereo cameras

- **arm_position_setter**
  - Subscribes: `/object_position`
  - Publishes: `/target_position` (Float32MultiArray)
  - Forwards detected position as target

- **inverse_kinematics**
  - Subscribes: `/target_position`
  - Publishes: `/joint_angles` (Int32MultiArray)
  - Computes joint angles for the robot arm

- **servo_commands**
  - Subscribes: `/joint_angles`
  - Sends commands to Arduino via serial

- **servo_gui** (optional)
  - Publishes: `/joint_angles` (manual override)
  - GUI for manual servo control

## Usage

1. Build the package with `colcon build` in your ROS2 workspace.
2. Source your workspace: `source install/setup.bash`
3. Run nodes as needed:
   - `ros2 run ros2_robot_arm stereo_vision_position_detection`
   - `ros2 run ros2_robot_arm arm_position_setter`
   - `ros2 run ros2_robot_arm inverse_kinematics`
   - `ros2 run ros2_robot_arm servo_commands`
   - `ros2 run ros2_robot_arm servo_gui` (optional)

## Dependencies
- Python 3
- OpenCV
- numpy
- rclpy
- std_msgs
- sensor_msgs

## Hardware
- Stereo cameras
- Arduino-controlled robot arm

## Notes
- Update serial port in `servo_commands.py` and `ik_test.py` as needed.
- Place calibration files (`cam_matrix.p`, `dist_matrix.p`) in the package directory.
