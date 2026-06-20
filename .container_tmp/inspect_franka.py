from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "fast_shutdown": True, "create_new_stage": False}, experience="/isaac-sim/apps/isaacsim.exp.base.python.kit")
try:
    import omni.kit.app, omni.usd
    from pathlib import Path
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    root=Path("/isaac-sim")
    ext=omni.kit.app.get_app().get_extension_manager()
    ext.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
    urdf_path=root / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"
    outdir=Path("/tmp/native_harvest_inspect")
    outdir.mkdir(exist_ok=True)
    importer=URDFImporter(); cfg=URDFImporterConfig(); cfg.urdf_path=str(urdf_path); cfg.usd_path=str(outdir); cfg.merge_mesh=False; cfg.collision_from_visuals=False; importer.config=cfg
    usd_path=importer.import_urdf()
    omni.usd.get_context().open_stage(usd_path)
    stage=omni.usd.get_context().get_stage()
    robot=stage.GetPrimAtPath("/panda_arm_hand")
    if robot.IsValid():
        robot.GetVariantSet("Physics").SetVariantSelection("physx")
    app.update(); app.update(); app.update()
    from pxr import UsdPhysics, PhysxSchema
    for prim in stage.Traverse():
        p=prim.GetPath().pathString
        if any(k in p.lower() for k in ["panda_hand", "leftfinger", "rightfinger"]):
            markers=[]
            if prim.HasAPI(UsdPhysics.RigidBodyAPI): markers.append("RigidBody")
            if prim.HasAPI(UsdPhysics.CollisionAPI): markers.append("Collision")
            if prim.HasAPI(UsdPhysics.MassAPI): markers.append("Mass")
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI): markers.append("PhysxRigid")
            print(p, prim.GetTypeName(), ",".join(markers))
finally:
    app.close()
