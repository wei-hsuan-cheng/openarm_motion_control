# Motion Control Simulation of OpenArmv10

## Installation

This is a ROS 2 Humble package.
> First download and install the robot_description of *openarm_v10* [here](https://github.com/enactic/openarm_description.git)!!!

```bash
cd ~/ros2_ws/src && git clone https://github.com/wei-hsuan-cheng/openarm_motion_control.git
cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select openarm_motion_control && . install/setup.bash
```

## Run Demo

```bash
cd ~/ros2_ws

# Visualize in RViz2 with joint_state_publisher_gui
ros2 launch openarm_motion_control visualize_openarm.launch.py \
arm_type:=v10 \
bimanual:=true

# Task space motion (IK pose command)
ros2 launch openarm_motion_control pose_command.launch.py \
arm_type:=v10 \
bimanual:=true

# Arguments:
# ee_type:=none # if no end-effector
# bimanual:=false # if single arm
```