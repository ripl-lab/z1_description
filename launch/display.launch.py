"""Show the Z1 with one end-of-arm variant in RViz, with a slider for each joint.

    ros2 launch z1_description display.launch.py variant:=bota_probe

Arguments: variant (unitree_gripper, bota_probe or robotiq_2f85), with_gripper, with_camera.
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory("z1_description"))


def _nodes(context):
    args = {name: LaunchConfiguration(name).perform(context) for name in ("variant", "with_gripper", "with_camera")}
    # sim_ignition false: no gz-sim plugin, so the controllers file of z1_bringup is not needed.
    urdf = xacro.process_file(str(SHARE / "urdf" / "z1.urdf.xacro"), mappings={**args, "sim_ignition": "false"}).toxml()
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(SHARE / "config" / "display.rviz")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("variant", default_value="unitree_gripper",
                              description="unitree_gripper, bota_probe or robotiq_2f85"),
        DeclareLaunchArgument("with_gripper", default_value="true", description="the gripper motor is wired"),
        DeclareLaunchArgument("with_camera", default_value="true", description="the variant's cameras"),
        OpaqueFunction(function=_nodes),
    ])
