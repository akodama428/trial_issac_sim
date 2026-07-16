from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "src/franka_ros2_control/config/franka_ros2_control.urdf"
SRDF = ROOT / "src/franka_ros2_control/config/panda.srdf"
BRINGUP = ROOT / "src/franka_ros2_control/launch/franka_ros2_control.launch.py"
RUNNER = ROOT / "scripts/run_ros2_components.sh"
ADAPTER = (
    ROOT
    / "src/tomato_harvest_sim/robot/execute_manager/servo_execution_adapter.py"
)


def test_canonical_robot_description_contains_link8_fixed_chain() -> None:
    robot = ET.fromstring(URDF.read_text(encoding="utf-8"))
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    assert "panda_link8" in links
    joint8 = joints["panda_joint8"]
    assert joint8.attrib["type"] == "fixed"
    assert joint8.find("parent").attrib["link"] == "panda_link7"
    assert joint8.find("child").attrib["link"] == "panda_link8"
    assert joint8.find("origin").attrib == {"xyz": "0 0 0.107", "rpy": "0 0 0"}
    hand_joint = joints["panda_hand_joint"]
    assert hand_joint.find("parent").attrib["link"] == "panda_link8"
    assert hand_joint.find("origin").attrib == {
        "xyz": "0 0 0",
        "rpy": "0 0 -0.7853981633974483",
    }


def test_robot_bringup_owns_standard_robot_state_publisher() -> None:
    launch = BRINGUP.read_text(encoding="utf-8")

    assert 'package="robot_state_publisher"' in launch
    assert 'executable="robot_state_publisher"' in launch
    assert "parameters=[robot_description]" in launch


def test_moveit_arm_tip_matches_tf_control_link() -> None:
    robot = ET.fromstring(SRDF.read_text(encoding="utf-8"))
    arm = next(group for group in robot.findall("group") if group.attrib["name"] == "panda_arm")

    assert arm.find("chain").attrib == {
        "base_link": "panda_link0",
        "tip_link": "panda_link8",
    }


def test_runner_uses_robot_bringup_instead_of_assembling_tf_process() -> None:
    runner = RUNNER.read_text(encoding="utf-8")

    assert "ros2 launch franka_ros2_control franka_ros2_control.launch.py" in runner
    assert "ros2 run robot_state_publisher robot_state_publisher" not in runner
    assert "ros2 run controller_manager ros2_control_node" not in runner
    assert "-name '*.srdf'" in runner


def test_servo_adapter_has_no_scene_snapshot_pose_fallback() -> None:
    adapter = ADAPTER.read_text(encoding="utf-8")

    assert "SCENE_SNAPSHOT_TOPIC" not in adapter
    assert "scene_snapshot_from_json" not in adapter
    assert "select_current_link_pose" not in adapter
    assert "_runtime_tool_pose" not in adapter
