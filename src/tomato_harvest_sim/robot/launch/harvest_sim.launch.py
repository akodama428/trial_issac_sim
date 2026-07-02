"""harvest_sim.launch.py

トマト収穫シミュレーション全体を起動するメインランチファイル。

起動順序:
  1. franka_ros2_control (C++ controller_manager + JTC + JSB)
  2. MoveIt2 move_group (オプション)
  3. tomato_harvest_robot_node  (Python ROS2 Timer 30 Hz)
  ※ tomato_harvest_simulator_node は Isaac Sim プロセスとして別途起動
     （Isaac Sim の Python 環境内で simulator_node.py を直接実行する）

使い方:
    ros2 launch tomato_harvest_robot harvest_sim.launch.py

Isaac Sim を先に起動する場合:
    python simulator_node.py  # Isaac Sim Python 環境内で実行
    ros2 launch tomato_harvest_robot harvest_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_moveit_arg = DeclareLaunchArgument(
        "use_moveit",
        default_value="false",
        description="MoveIt2 move_group を起動するか",
    )

    robot_node = Node(
        package="tomato_harvest_robot",
        executable="tomato_harvest_robot_node",
        name="tomato_harvest_robot_node",
        output="screen",
    )

    franka_launch_path = os.path.join(
        os.path.dirname(__file__),
        "franka_controllers.launch.py",
    )

    franka_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(franka_launch_path),
    )

    return LaunchDescription(
        [
            use_moveit_arg,
            franka_controllers,
            # robot_node は franka 起動後に開始（controller が ready になるまで待つ）
            TimerAction(
                period=5.0,
                actions=[robot_node],
            ),
        ]
    )
