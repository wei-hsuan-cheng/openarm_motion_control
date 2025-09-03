# Motion Control Simulation of OpenArmv10

## Run Demo

### Single arm w/o gripper

```bash
# Joint space motion
ros2 launch openarm_motion_control joint_command.launch.py arm_type:=v10 bimanual:=false ee_type:=none
```