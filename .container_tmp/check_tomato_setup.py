from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "fast_shutdown": True, "create_new_stage": False, "disable_viewport_updates": True}, experience="/isaac-sim/apps/isaacsim.exp.base.python.kit")
try:
    import sys
    sys.path.insert(0, "/workspace/tomato-harvest/src")
    from tomato_harvest_poc.isaac_native_runtime import IsaacNativeRuntime
    rt = IsaacNativeRuntime(headless=True)
    rt._simulation_app = app
    rt._setup_runtime()
    print("tomato_world=", rt._read_tomato_world_position())
    print("hand_cam_tomato=", rt._read_tomato_positions()[0])
    print("stem_joint_active=", rt._fruit_stem_joint_active)
    print("tray_paths=", rt._tray_contact_paths)
    rt._reset_scene(reset_phase=False)
    print("after_reset_tomato_world=", rt._read_tomato_world_position())
finally:
    app.close()
