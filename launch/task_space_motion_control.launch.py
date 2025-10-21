import os
import yaml
import xacro
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

""" 
Load yaml files and params
"""
yaml_dir = os.path.join(
    get_package_share_directory("openarm_motion_control"),
)
left_yaml = yaml_dir + "/screw_lists/openarm_v10_left_body_screws.yaml"
right_yaml = yaml_dir + "/screw_lists/openarm_v10_right_body_screws.yaml"

initial_cfg_path = os.path.join(
    get_package_share_directory("openarm_motion_control"),
    "config", "initial_configuration.yaml"
)

motion_params_left = {
    "pos_x_cmd": 0.325, # 0.25, 0.325, 0.45
    "pos_y_cmd": 0.15,
    "pos_z_cmd": 0.5,
    "quat_w_cmd": -0.085,
    "quat_x_cmd": -0.191,
    "quat_y_cmd": 0.879,
    "quat_z_cmd": -0.429,
    
    "offset_rad": [0.0],
    "amplitude_rad": [0.35],
    "frequency_hz": [0.05],
    "phase_rad": [0.0],
}

motion_params_right = {
    "pos_x_cmd": 0.20,
    "pos_y_cmd": -0.25,
    "pos_z_cmd": 0.40,
    "quat_w_cmd": -0.085,
    "quat_x_cmd": 0.191,
    "quat_y_cmd": 0.879,
    "quat_z_cmd": 0.429,
    
    "offset_rad": [0.0],
    "amplitude_rad": [0.35],
    "frequency_hz": [0.01],
    "phase_rad": [0.0],
}
    
def _load_yaml_dict(path: str) -> dict:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML at {path} is not a dict at top level.")
    return data

def _load_screw_list_params(yaml_path_abs: str) -> dict:
    params_raw = _load_yaml_dict(yaml_path_abs)
    robot_name = params_raw.get("robot_name", "arm")
    jl = params_raw.get("joint_limits", {})
    names = params_raw.get("joint_names", [])
    jl_lower  = []
    jl_upper  = []
    jl_vel    = []
    jl_effort = []
    for name in names:
        d = jl.get(name, {})
        jl_lower.append( float(d.get("lower",  0.0)) )
        jl_upper.append( float(d.get("upper",  0.0)) )
        jl_vel.append(   float(d.get("velocity", 0.0)) )
        jl_effort.append(float(d.get("effort",  0.0)) )

    screw_list_params = {
        "robot_name":           robot_name,
        "base_link":            params_raw.get("base_link", ""),
        "ee_link":              params_raw.get("ee_link", ""),
        "screw_representation": params_raw.get("screw_representation", "body"),
        "joint_names":          names,
        "num_joints":           params_raw.get("num_joints", 0),
        "screw_list":           params_raw.get("screw_list", {}),
        "joint_limits_lower":   jl_lower,
        "joint_limits_upper":   jl_upper,
        "joint_limits_velocity": jl_vel,
        "joint_limits_effort":  jl_effort,
        "M_position":           params_raw.get("M_position", []),
        "M_quaternion_wxyz":    params_raw.get("M_quaternion_wxyz", []),
    }
    return screw_list_params

def robot_state_publisher_spawner(context: LaunchContext, arm_type, ee_type, bimanual):
    arm_type_str = context.perform_substitution(arm_type)
    ee_type_str = context.perform_substitution(ee_type)
    bimanual_str = context.perform_substitution(bimanual)

    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", f"{arm_type_str}.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "ee_type": ee_type_str,
            "bimanual": bimanual_str,
        }
    ).toprettyxml(indent="  ")

    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )]


def rviz_spawner(context: LaunchContext):
    rviz_config_file = "motion_control.rviz"
    rviz_config_path = os.path.join(
        get_package_share_directory("openarm_motion_control"),
        "rviz", rviz_config_file
    )
    return [Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config_path],
        output="screen",
    )]

def motion_reference_generator_spawner(context: LaunchContext) -> List[Node]:
    mrg_left  = Node(
        package="openarm_motion_control",
        executable="motion_reference_generator",
        name="motion_reference_generator",
        output="screen",
        parameters=[_load_screw_list_params(left_yaml),
                    {"gripper_joint_name": "openarm_left_finger_joint1"},
                    initial_cfg_path,
                    {"fs": 200.0},
                    motion_params_left],
        remappings=[("/pose_command", "/openarm_left/pose_command"),
                    ("/joint_command", "/openarm_left/joint_command")],
    )

    mrg_right = Node(
        package="openarm_motion_control",
        executable="motion_reference_generator",
        name="motion_reference_generator",
        output="screen",
        parameters=[_load_screw_list_params(right_yaml),
                    {"gripper_joint_name": "openarm_right_finger_joint1"},
                    initial_cfg_path,
                    {"fs": 200.0},
                    motion_params_right],
        remappings=[("/pose_command", "/openarm_right/pose_command"),
                    ("/joint_command", "/openarm_right/joint_command")],
    )
    
    pvr_right = Node(
        package="openarm_motion_control",
        executable="pose_visualization_right",
        name="pose_visualization_right",
        output="screen",
        parameters=[_load_screw_list_params(right_yaml),
                    initial_cfg_path,
                    {"fs": 200.0},
                    motion_params_right],
    )
    
    return [
            mrg_left, 
            mrg_right,
            # pvr_right,
            ]

def motion_control_spawner(context: LaunchContext) -> List[Node]:
    # Task Space Motion Control
    tsmc_left = Node(
        package="openarm_motion_control",
        executable="task_space_motion_control",
        name="task_space_motion_control",
        output="screen",
        parameters=[_load_screw_list_params(left_yaml),
                    initial_cfg_path,
                    {"fs": 200.0}],
        remappings=[("/pose_command", "/openarm_left/pose_command"),
                    ("/joint_velocity_command", "/openarm_left/joint_velocity_command"),],
    )
    
    tsmc_right = Node(
        package="openarm_motion_control",
        executable="task_space_motion_control",
        name="task_space_motion_control",
        output="screen",
        parameters=[_load_screw_list_params(right_yaml),
                    initial_cfg_path,
                    {"fs": 200.0}],
        remappings=[("/pose_command", "/openarm_right/pose_command"),
                    ("/joint_velocity_command", "/openarm_right/joint_velocity_command"),],
    )

    return [
            tsmc_left, 
            # tsmc_right,
            ]

def robot_joint_dynamics_spawner(context: LaunchContext) -> List[Node]:
    rjd_left = Node(
        package="openarm_motion_control",
        executable="robot_joint_dynamics",
        name="robot_joint_dynamics",
        output="screen",
        parameters=[_load_screw_list_params(left_yaml),
                    initial_cfg_path,
                    {"fs": 500.0},],
        remappings=[("/joint_velocity_command", "/openarm_left/joint_velocity_command")],
    )
    
    rjd_right = Node(
        package="openarm_motion_control",
        executable="robot_joint_dynamics",
        name="robot_joint_dynamics",
        output="screen",
        parameters=[_load_screw_list_params(right_yaml),
                    initial_cfg_path,
                    {"fs": 500.0},],
        remappings=[("/joint_velocity_command", "/openarm_right/joint_velocity_command")],
    )
    
    return [
            rjd_left, 
            # rjd_right,
            ]

def generate_launch_description():
    # Args
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="v10",
        description="Type of arm to visualize (e.g., v10)"
    )
    ee_type_arg = DeclareLaunchArgument(
        "ee_type",
        default_value="openarm_hand",
        description="Type of end-effector to attach (e.g., openarm_hand or none)"
    )
    bimanual_arg = DeclareLaunchArgument(
        "bimanual",
        default_value="true",
        description="Whether to use bimanual configuration"
    )

    # LaunchConfigurations
    arm_type = LaunchConfiguration("arm_type")
    ee_type = LaunchConfiguration("ee_type")
    bimanual = LaunchConfiguration("bimanual")

    # Runtime-resolved spawners
    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type, bimanual]
    )
    rviz_loader = OpaqueFunction(
        function=rviz_spawner,
        args=[]
    )
    motion_reference_generator_loader = OpaqueFunction(
        function=motion_reference_generator_spawner,
        args=[]
    )
    motion_control_loader = OpaqueFunction(
        function=motion_control_spawner,
        args=[]
    )
    robot_joint_dynamics_loader = OpaqueFunction(
        function=robot_joint_dynamics_spawner,
        args=[]
    )

    return LaunchDescription([
        arm_type_arg,
        ee_type_arg,
        bimanual_arg,
        robot_state_publisher_loader,
        rviz_loader,
        motion_reference_generator_loader,
        motion_control_loader,
        robot_joint_dynamics_loader,
    ])
