from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'fast_shutdown': True})
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
solver = LulaKinematicsSolver(
    robot_description_path='/isaac-sim/extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml',
    urdf_path='/isaac-sim/extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf',
)
print('\n'.join(solver.get_all_frame_names()))
app.close()
