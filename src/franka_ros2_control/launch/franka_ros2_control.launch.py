from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("franka_ros2_control")

    # franka_controllers.yaml の controller 名と一致させること。
    joint_trajectory_controller_name = "joint_trajectory_controller"

    urdf_path = PathJoinSubstitution([pkg_share, "config", "franka_ros2_control.urdf"])
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "franka_controllers.yaml"])

    robot_description = {"robot_description": open(
        Path(__file__).parent.parent / "config" / "franka_ros2_control.urdf"
    ).read()}

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[joint_trajectory_controller_name, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    motion_command_executor = Node(
        package="franka_ros2_control",
        executable="motion_command_executor_node",
        name="motion_command_executor_node",
        parameters=[{
            "follow_joint_trajectory_action":
                f"/{joint_trajectory_controller_name}/follow_joint_trajectory",
        }],
        output="screen",
    )

    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        motion_command_executor,
    ])
