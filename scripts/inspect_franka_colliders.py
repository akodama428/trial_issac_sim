#!/usr/bin/env python3
"""franka.usd の finger / hand collider を検査する（Step 1、計画 §9.1-R5）。

公式 Franka USD アセットのグリッパー周辺 prim について、collider の有無・
形状タイプ・近似方式・既存物理マテリアルを列挙し、摩擦把持に使える
コリジョン構成かを確認する。

実行（コンテナ内）:
  PYTHONPATH=/workspace/tomato-harvest/src ./python.sh scripts/inspect_franka_colliders.py
"""
from __future__ import annotations


def main() -> None:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import omni.usd
        from pxr import Usd, UsdPhysics, UsdShade

        from tomato_harvest_sim.simulator.isaac_viewer import (
            build_official_franka_asset_path,
        )
        from isaacsim.storage.native import get_assets_root_path

        assets_root = get_assets_root_path()
        asset_url = build_official_franka_asset_path(assets_root)
        print(f"[InspectColliders] asset={asset_url}", flush=True)

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        robot_prim = stage.DefinePrim("/World/FrankaPanda")
        robot_prim.GetReferences().AddReference(asset_url)

        targets = ("panda_leftfinger", "panda_rightfinger", "panda_hand")
        # franka.usd は instanceable 参照を含むため、インスタンスプロキシも辿る
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if not any(target in path for target in targets):
                continue
            has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)
            has_mesh_collision = prim.HasAPI(UsdPhysics.MeshCollisionAPI)
            approximation = ""
            if has_mesh_collision:
                mesh_api = UsdPhysics.MeshCollisionAPI(prim)
                approximation = str(mesh_api.GetApproximationAttr().Get())
            bound_material = ""
            binding_api = UsdShade.MaterialBindingAPI(prim)
            binding = binding_api.GetDirectBinding(materialPurpose="physics")
            if binding.GetMaterial():
                bound_material = str(binding.GetMaterialPath())
            if has_collision or has_mesh_collision or prim.GetTypeName() in ("Mesh", "Xform"):
                print(
                    "[InspectColliders] "
                    f"path={path} type={prim.GetTypeName()} "
                    f"collision={int(has_collision)} "
                    f"mesh_approx={approximation or 'n/a'} "
                    f"physics_material={bound_material or 'none'}",
                    flush=True,
                )
        print("[InspectColliders] done", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
