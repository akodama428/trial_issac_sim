from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from tomato_harvest_sim.robot.motion_planner.ros_python import ensure_ros_python_modules_available


MOVE_GROUP_EXECUTABLE = "/opt/ros/jazzy/lib/moveit_ros_move_group/move_group"
ISAAC_FRANKA_PACKAGE_ROOT = (
    "/isaac-sim/exts/isaacsim.asset.importer.urdf/data/urdf/robots"
)
ISAAC_PANDA_URDF = (
    "/isaac-sim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/"
    "franka_description/robots/panda_arm_hand.urdf"
)


class _MoveItParamsDumper(yaml.SafeDumper):
    pass


def _represent_multiline_string(dumper: yaml.SafeDumper, data: str) -> yaml.nodes.ScalarNode:
    style = "|" if "\n" in data else None
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style=style)


_MoveItParamsDumper.add_representer(str, _represent_multiline_string)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _config_root() -> Path:
    return Path(__file__).resolve().parents[1] / "moveit_config"


def _load_text(path: Path | str) -> str:
    return Path(path).read_text(encoding="utf-8")


def _load_yaml(path: Path | str) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def _planning_pipeline_config() -> dict[str, Any]:
    return {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                }
            },
            "panda_arm": {
                "planner_configs": ["RRTConnectkConfigDefault"],
                "projection_evaluator": "joints(panda_joint1,panda_joint2)",
                "longest_valid_segment_fraction": 0.01,
            },
        },
    }


def build_move_group_parameters() -> dict[str, Any]:
    config_root = _config_root()
    return {
        "robot_description": _load_text(ISAAC_PANDA_URDF),
        "robot_description_semantic": _load_text(config_root / "panda.srdf"),
        "robot_description_kinematics": _load_yaml(config_root / "kinematics.yaml"),
        "robot_description_planning": _load_yaml(config_root / "joint_limits.yaml"),
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "allow_trajectory_execution": False,
        "capabilities": "",
        "disable_capabilities": "",
        "octomap_frame": "panda_link0",
        "octomap_resolution": 0.05,
        "max_safe_path_cost": 1.0,
        "jiggle_fraction": 0.02,
        "max_sampling_attempts": 16,
        **_planning_pipeline_config(),
    }


def write_move_group_parameters_file(directory: Path | str) -> Path:
    target_dir = Path(directory)
    target_dir.mkdir(parents=True, exist_ok=True)
    params_path = target_dir / "move_group.params.yaml"
    payload = {"move_group": {"ros__parameters": build_move_group_parameters()}}
    params_path.write_text(
        yaml.dump(payload, Dumper=_MoveItParamsDumper, sort_keys=False),
        encoding="utf-8",
    )
    return params_path


def _service_names() -> tuple[str, str]:
    plan_name = os.environ.get("TOMATO_HARVEST_MOVEIT_SERVICE", "/plan_kinematic_path")
    scene_name = os.environ.get("TOMATO_HARVEST_MOVEIT_SCENE_SERVICE", "/apply_planning_scene")
    return plan_name, scene_name


def moveit_service_available(*, timeout_sec: float = 0.5) -> bool:
    if not ensure_ros_python_modules_available("rclpy", "moveit_msgs"):
        return False
    try:
        import rclpy
        from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
    except Exception:
        return False

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True

    node = rclpy.create_node("tomato_harvest_moveit_service_probe")
    try:
        plan_name, scene_name = _service_names()
        plan_client = node.create_client(GetMotionPlan, plan_name)
        scene_client = node.create_client(ApplyPlanningScene, scene_name)
        return bool(plan_client.wait_for_service(timeout_sec=timeout_sec)) and bool(
            scene_client.wait_for_service(timeout_sec=timeout_sec)
        )
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


@dataclass
class MoveItServiceManager:
    process: subprocess.Popen[str] | None
    params_path: Path | None
    log_path: Path | None

    @classmethod
    def start_if_needed(cls) -> "MoveItServiceManager":
        if moveit_service_available(timeout_sec=0.2):
            return cls(process=None, params_path=None, log_path=None)

        temp_dir = Path(tempfile.mkdtemp(prefix="tomato-moveit-", dir="/tmp"))
        log_path = temp_dir / "move_group.log"
        params_path = write_move_group_parameters_file(temp_dir)

        env = os.environ.copy()
        ros_package_path = env.get("ROS_PACKAGE_PATH", "")
        if ISAAC_FRANKA_PACKAGE_ROOT not in ros_package_path.split(":"):
            env["ROS_PACKAGE_PATH"] = (
                f"{ISAAC_FRANKA_PACKAGE_ROOT}:{ros_package_path}" if ros_package_path else ISAAC_FRANKA_PACKAGE_ROOT
            )
        python_path = env.get("PYTHONPATH", "")
        src_root = str(_repo_root() / "src")
        if src_root not in python_path.split(":"):
            env["PYTHONPATH"] = f"{src_root}:{python_path}" if python_path else src_root

        process = subprocess.Popen(
            [
                MOVE_GROUP_EXECUTABLE,
                "--ros-args",
                "--params-file",
                str(params_path),
            ],
            stdout=log_path.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
        )

        deadline = time.time() + float(os.environ.get("TOMATO_HARVEST_MOVEIT_START_TIMEOUT_SEC", "15.0"))
        while time.time() < deadline:
            if process.poll() is not None:
                break
            if moveit_service_available(timeout_sec=0.2):
                return cls(process=process, params_path=params_path, log_path=log_path)
            time.sleep(0.2)

        process.terminate()
        process.wait(timeout=5)
        raise RuntimeError(
            "Failed to start move_group service. "
            f"See log: {log_path}"
        )

    def shutdown(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)


def main() -> int:
    temp_dir = Path(tempfile.mkdtemp(prefix="tomato-moveit-manual-", dir="/tmp"))
    params_path = write_move_group_parameters_file(temp_dir)
    process = subprocess.Popen(
        [
            MOVE_GROUP_EXECUTABLE,
            "--ros-args",
            "--params-file",
            str(params_path),
        ],
        text=True,
    )
    try:
        return int(process.wait())
    except KeyboardInterrupt:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        return 130
