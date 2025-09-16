# Motion Control Simulation of OpenArmv10

## Installation

This is a ROS 2 Humble package.

```bash
cd ~/ros2_ws/src && git clone https://github.com/wei-hsuan-cheng/openarm_motion_control.git
cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select openarm_motion_control && . install/setup.bash
```

## Run Demo

### Single arm w/o gripper

```bash
cd ~/ros2_ws
# Visualize in RViz2 with joint_state_publisher_gui
ros2 launch openarm_motion_control visualize_openarm.launch.py \
arm_type:=v10 \
bimanual:=false
# ee_type:=none

# Joint space motion
ros2 launch openarm_motion_control joint_command.launch.py \
arm_type:=v10 \
bimanual:=false

# Task space motion
ros2 launch openarm_motion_control pose_command.launch.py \
arm_type:=v10 \
bimanual:=false
```