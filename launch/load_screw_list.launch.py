import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _load_yaml_dict(path: str) -> dict:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML at {path} is not a dict at top level.")
    return data

def generate_launch_description():
    default_yaml = os.path.expanduser(
        "~/ros2_ws/openarm_ws/src/openarm_motion_control/screw_lists/openarm_v10_body_screws.yaml"
    )
    yaml_path_arg = DeclareLaunchArgument(
        "yaml_path",
        default_value=default_yaml,
        description="Absolute path to openarm screw-list YAML",
    )

    yaml_path = LaunchConfiguration("yaml_path")
    params_raw = _load_yaml_dict(default_yaml)

    screw_list_params = {
        "base_link":              params_raw.get("base_link", ""),
        "ee_link":                params_raw.get("ee_link", ""),
        "screw_representation":   params_raw.get("screw_representation", "body"),
        "joint_names":            params_raw.get("joint_names", []),
        "num_joints":             params_raw.get("num_joints", 0),
        "screw_list":             params_raw.get("screw_list", {}),           # dict: joint_name -> [6 doubles]
        "M_position":             params_raw.get("M_position", {}),           # list or dict both OK
        "M_quaternion_wxyz":      params_raw.get("M_quaternion_wxyz", {}),    # list [w,x,y,z]
    }

    load_screw_list_node = Node(
        package="openarm_motion_control",
        executable="load_screw_list",
        name="load_screw_list",
        output="screen",
        parameters=[screw_list_params],
    )

    return LaunchDescription([
        yaml_path_arg,
        load_screw_list_node,
    ])
