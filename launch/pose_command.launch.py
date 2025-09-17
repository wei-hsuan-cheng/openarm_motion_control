# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import xacro
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _load_yaml_dict(path: str) -> dict:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML at {path} is not a dict at top level.")
    return data


def _pose_node_from_yaml(yaml_path_abs: str, node_name: str) -> Node:
    """Make a pose_command node from a YAML; preserves your param schema."""
    params_raw = _load_yaml_dict(yaml_path_abs)

    screw_list_params = {
        "base_link":            params_raw.get("base_link", ""),
        "ee_link":              params_raw.get("ee_link", ""),
        "screw_representation": params_raw.get("screw_representation", "body"),
        "joint_names":          params_raw.get("joint_names", []),
        "num_joints":           params_raw.get("num_joints", 0),
        "screw_list":           params_raw.get("screw_list", {}),
        "M_position":           params_raw.get("M_position", []),         # list expected
        "M_quaternion_wxyz":    params_raw.get("M_quaternion_wxyz", []),  # list expected
    }

    motion_params = {
        "fs": 200.0,
        "offset_rad": [0.0],
        "amplitude_rad": [0.35],
        "frequency_hz": [0.1],
        "phase_rad": [0.0],
    }

    return Node(
        package="openarm_motion_control",
        executable=node_name,
        name=node_name,
        output="screen",
        parameters=[screw_list_params, motion_params],
    )


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


def rviz_spawner(context: LaunchContext, bimanual):
    bimanual_str = context.perform_substitution(bimanual)
    rviz_config_file = "pose_command_bimanual.rviz" if bimanual_str.lower() == "true" else "pose_command_single_arm.rviz"
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


def pose_command_spawner(context: LaunchContext, bimanual, yaml_path) -> List[Node]:
    """
    If bimanual=false:
      - spawn one node 'pose_command' using yaml_path (…/openarm_v10_body_screws.yaml)
    If bimanual=true:
      - spawn two nodes:
          * 'pose_command_left'  using DIR/openarm_v10_left_body_screws.yaml
          * 'pose_command_right' using DIR/openarm_v10_right_body_screws.yaml
        where DIR is the directory of yaml_path (so you can relocate the set easily).
    """
    bimanual_str = context.perform_substitution(bimanual)
    is_bimanual = (bimanual_str.lower() == "true")

    yaml_resolved = os.path.expanduser(context.perform_substitution(yaml_path))
    yaml_dir = os.path.dirname(yaml_resolved)

    # single arm
    if not is_bimanual: 
        return [_pose_node_from_yaml(yaml_resolved, "pose_command")]

    # bimanual: enforce left/right files in the same directory
    left_yaml  = os.path.join(yaml_dir, "openarm_v10_left_body_screws.yaml")
    right_yaml = os.path.join(yaml_dir, "openarm_v10_right_body_screws.yaml")
    left_node  = _pose_node_from_yaml(left_yaml,  "pose_command_left")
    right_node = _pose_node_from_yaml(right_yaml, "pose_command_right")
    return [left_node, right_node]


def generate_launch_description():
    # Args
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        description="Type of arm to visualize (e.g., v10)"
    )
    ee_type_arg = DeclareLaunchArgument(
        "ee_type",
        default_value="openarm_hand",
        description="Type of end-effector to attach (e.g., openarm_hand or none)"
    )
    bimanual_arg = DeclareLaunchArgument(
        "bimanual",
        default_value="false",
        description="Whether to use bimanual configuration"
    )

    # Default YAML (single-arm). For bimanual we’ll swap to left/right in the same directory.
    default_yaml = os.path.expanduser(
        "~/ros2_ws/openarm_ws/src/openarm_motion_control/screw_lists/openarm_v10_body_screws.yaml"
    )
    yaml_path_arg = DeclareLaunchArgument(
        "yaml_path",
        default_value=default_yaml,
        description="Path to single-arm YAML (its directory will also be used to find the left/right YAMLs when bimanual=true)",
    )

    # LaunchConfigurations
    arm_type = LaunchConfiguration("arm_type")
    ee_type = LaunchConfiguration("ee_type")
    bimanual = LaunchConfiguration("bimanual")
    yaml_path = LaunchConfiguration("yaml_path")

    # Runtime-resolved spawners
    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type, bimanual]
    )
    rviz_loader = OpaqueFunction(
        function=rviz_spawner,
        args=[bimanual]
    )
    pose_command_loader = OpaqueFunction(
        function=pose_command_spawner,
        args=[bimanual, yaml_path]
    )

    return LaunchDescription([
        arm_type_arg,
        ee_type_arg,
        bimanual_arg,
        yaml_path_arg,
        robot_state_publisher_loader,
        rviz_loader,
        pose_command_loader,
    ])
