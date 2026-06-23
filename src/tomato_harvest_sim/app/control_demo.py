from __future__ import annotations

from tomato_harvest_sim.api.contracts import ControlCommand
from tomato_harvest_sim.app.application import create_tomato_harvest_application


def main() -> None:
    system = create_tomato_harvest_application()
    try:
        system.boot()
        print("[boot] simulator=ready robot=ready camera=fixed_camera")

        for command in (ControlCommand.START, ControlCommand.STOP, ControlCommand.RESET):
            result = system.apply_control(command)
            print(
                f"[{command.value}] accepted={result.accepted} "
                f"scene={result.scene_phase.value} "
                f"robot={result.robot_state.value}"
            )
    finally:
        system.close()


if __name__ == "__main__":
    main()
