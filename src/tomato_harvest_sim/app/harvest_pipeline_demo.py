from __future__ import annotations

import argparse

from tomato_harvest_sim.api.contracts import ControlCommand
from tomato_harvest_sim.app.application import create_tomato_harvest_application


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tomato harvest robot pipeline terminal demo")
    parser.add_argument(
        "--grasp-mode",
        choices=("success", "failure"),
        default="success",
        help="Choose the stable grasp or failed grasp demo path.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=64,
        help="Number of runtime steps to execute.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    system = create_tomato_harvest_application(grasp_mode=args.grasp_mode)
    try:
        system.boot()
        print("[boot] scene=ready robot=ready task=idle")

        result = system.apply_control(ControlCommand.START)
        print(
            f"[start] accepted={result.accepted} "
            f"scene={result.scene_phase.value} robot={result.robot_state.value}"
        )

        for step_index in range(1, args.steps + 1):
            logs = system.step()
            if logs:
                print(f"[step {step_index}]")
                for line in logs:
                    print(f"  {line}")

        print(
            "[final] "
            f"task={system.robot.state.task_phase.value} "
            f"tomato_status={system.simulator.state.tomato_status.value} "
            f"tool_xyz=({system.simulator.state.robot_tool_pose.x:.2f},"
            f"{system.simulator.state.robot_tool_pose.y:.2f},"
            f"{system.simulator.state.robot_tool_pose.z:.2f})"
        )
    finally:
        system.close()


if __name__ == "__main__":
    main()
