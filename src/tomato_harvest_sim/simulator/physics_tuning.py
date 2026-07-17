"""Step 1（物理モデル土台）: 摩擦マテリアル・コリジョン・ソルバ設定の USD 適用。

scene.yaml の physics セクション（PhysicsTuningConfig）を stage 上の prim へ
反映する。設定値の正本は yaml であり、本モジュールは適用手段のみを持つ。
enabled=False のときは何も適用しない（従来挙動）。
"""
from __future__ import annotations

from tomato_harvest_sim.simulator.scene_config import (
    PhysicsMaterialConfig,
    PhysicsTuningConfig,
)

_MATERIAL_ROOT_PRIM_PATH = "/World/PhysicsMaterials"


def apply_physics_tuning(
    *,
    stage: object,
    config: PhysicsTuningConfig,
    tomato_prim_path: str,
    tomato_collision_prim_path: str,
    finger_link_prim_paths: tuple[str, ...],
    container_prim_paths: tuple[str, ...],
) -> list[str]:
    """物理チューニング一式を stage へ適用し、適用内容の記録を返す。

    Args:
        stage: USD stage。
        config: scene.yaml から読み込んだチューニング設定。
        tomato_prim_path: トマト剛体ルート prim（ソルバ設定の適用先）。
        tomato_collision_prim_path: トマト collider prim（オフセット・ねじり摩擦・
            マテリアルの適用先）。
        finger_link_prim_paths: 左右 finger link prim（グリッパーマテリアルの適用先）。
        container_prim_paths: トレイ・地面など静的 collider prim。

    Returns:
        適用内容を表す文字列リスト（検証レポート・ログ用）。未適用なら空。
    """
    if not config.enabled:
        return []

    applied: list[str] = []
    tomato_material = _define_physics_material(
        stage, f"{_MATERIAL_ROOT_PRIM_PATH}/Tomato", config.tomato_material, config
    )
    gripper_material = _define_physics_material(
        stage, f"{_MATERIAL_ROOT_PRIM_PATH}/Gripper", config.gripper_material, config
    )
    container_material = _define_physics_material(
        stage, f"{_MATERIAL_ROOT_PRIM_PATH}/Container", config.container_material, config
    )
    applied.append(
        f"materials defined at {_MATERIAL_ROOT_PRIM_PATH} "
        f"(frictionCombine={config.friction_combine_mode}, "
        f"restitutionCombine={config.restitution_combine_mode})"
    )

    if _bind_physics_material(stage, tomato_collision_prim_path, tomato_material):
        applied.append(f"tomato material bound: {tomato_collision_prim_path}")
    for finger_path in finger_link_prim_paths:
        if _bind_physics_material(stage, finger_path, gripper_material):
            applied.append(f"gripper material bound: {finger_path}")
    for container_path in container_prim_paths:
        if _bind_physics_material(stage, container_path, container_material):
            applied.append(f"container material bound: {container_path}")

    if _apply_tomato_collision_offsets(stage, tomato_collision_prim_path, config):
        applied.append(
            f"tomato collision offsets: contact={config.tomato_contact_offset_m} "
            f"rest={config.tomato_rest_offset_m} "
            f"torsionalPatch={config.tomato_torsional_patch_radius_m}"
        )

    if _apply_tomato_solver_iterations(stage, tomato_prim_path, config):
        applied.append(
            f"tomato solver iterations: pos={config.tomato_solver_position_iterations} "
            f"vel={config.tomato_solver_velocity_iterations}"
        )

    if _apply_tomato_angular_damping(stage, tomato_prim_path, config):
        applied.append(f"tomato angular damping: {config.tomato_angular_damping}")

    for finger_path in finger_link_prim_paths:
        if _apply_tomato_solver_iterations(stage, finger_path, config):
            applied.append(
                f"finger solver iterations: {finger_path} "
                f"pos={config.tomato_solver_position_iterations} "
                f"vel={config.tomato_solver_velocity_iterations}"
            )

    applied.extend(
        _apply_finger_drive_limits(stage, finger_link_prim_paths, config)
    )
    return applied


def _finger_joint_prim_paths(finger_link_prim_paths: tuple[str, ...]) -> tuple[str, ...]:
    """finger link パスから finger joint prim パスを導出する。

    franka.usd の構造では finger joint は panda_hand 配下にある:
      /World/FrankaPanda/panda_hand/panda_finger_joint1（左）/ joint2（右）
    """
    joints: list[str] = []
    for link_path in finger_link_prim_paths:
        if "panda_leftfinger" in link_path:
            joints.append(
                link_path.replace("panda_leftfinger", "panda_hand/panda_finger_joint1")
            )
        elif "panda_rightfinger" in link_path:
            joints.append(
                link_path.replace("panda_rightfinger", "panda_hand/panda_finger_joint2")
            )
    return tuple(joints)


def _apply_finger_drive_limits(
    stage: object,
    finger_link_prim_paths: tuple[str, ...],
    config: PhysicsTuningConfig,
) -> list[str]:
    """finger prismatic joint の drive を力制限付きバネへ設定する（Step 2）。

    HWI からの位置目標はそのまま drive の targetPosition として機能し、
    押付け力は maxForce で飽和する（UsdPhysics.DriveAPI の仕様、計画 §9.1-R4）。
    実アセットでは joint1 のみ drive を持ち joint2 は mimic 従動のため、
    drive が無い joint に mimic API があればそのまま尊重し、無ければ drive を新設する。
    max_force_n=0 は「適用しない」を意味する。
    """
    if config.finger_drive_max_force_n <= 0.0:
        return []
    from pxr import PhysxSchema, UsdPhysics

    applied: list[str] = []
    for joint_path in _finger_joint_prim_paths(finger_link_prim_paths):
        prim = stage.GetPrimAtPath(joint_path)
        if not prim.IsValid():
            applied.append(f"finger drive skipped (prim not found): {joint_path}")
            continue
        has_drive = prim.HasAPI(UsdPhysics.DriveAPI, "linear")
        has_mimic = prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
        if not has_drive and has_mimic:
            applied.append(f"finger drive skipped (mimic joint): {joint_path}")
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
        drive.CreateStiffnessAttr().Set(config.finger_drive_stiffness)
        drive.CreateDampingAttr().Set(config.finger_drive_damping)
        drive.CreateMaxForceAttr().Set(config.finger_drive_max_force_n)
        applied.append(
            f"finger drive set: {joint_path} "
            f"(stiffness={config.finger_drive_stiffness}, "
            f"damping={config.finger_drive_damping}, "
            f"maxForce={config.finger_drive_max_force_n}N, "
            f"pre_existing_drive={int(has_drive)}, mimic={int(has_mimic)})"
        )
    return applied


def _define_physics_material(
    stage: object,
    material_prim_path: str,
    material: PhysicsMaterialConfig,
    config: PhysicsTuningConfig,
) -> object:
    """摩擦・反発と combine mode を持つ物理マテリアル prim を定義する。"""
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    shade_material = UsdShade.Material.Define(stage, material_prim_path)
    material_api = UsdPhysics.MaterialAPI.Apply(shade_material.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(material.static_friction)
    material_api.CreateDynamicFrictionAttr().Set(material.dynamic_friction)
    material_api.CreateRestitutionAttr().Set(material.restitution)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(shade_material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr().Set(config.friction_combine_mode)
    physx_material.CreateRestitutionCombineModeAttr().Set(config.restitution_combine_mode)
    return shade_material


def _bind_physics_material(stage: object, prim_path: str, material: object) -> bool:
    """prim へ物理 purpose でマテリアルをバインドする（配下の collider に継承される）。"""
    from pxr import UsdShade

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return False
    binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
    binding_api.Bind(
        material,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics",
    )
    return True


def _apply_tomato_collision_offsets(
    stage: object, collision_prim_path: str, config: PhysicsTuningConfig
) -> bool:
    """小径球向けの contact/rest offset とねじり摩擦パッチ半径を設定する。"""
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(collision_prim_path)
    if not prim.IsValid():
        return False
    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision_api.CreateContactOffsetAttr().Set(config.tomato_contact_offset_m)
    collision_api.CreateRestOffsetAttr().Set(config.tomato_rest_offset_m)
    collision_api.CreateTorsionalPatchRadiusAttr().Set(config.tomato_torsional_patch_radius_m)
    collision_api.CreateMinTorsionalPatchRadiusAttr().Set(
        config.tomato_min_torsional_patch_radius_m
    )
    return True


def _apply_tomato_solver_iterations(
    stage: object, tomato_prim_path: str, config: PhysicsTuningConfig
) -> bool:
    """トマト剛体のみソルバ反復回数を増強する（シーン全体には波及させない）。"""
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(tomato_prim_path)
    if not prim.IsValid():
        return False
    rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rigid_body_api.CreateSolverPositionIterationCountAttr().Set(
        config.tomato_solver_position_iterations
    )
    rigid_body_api.CreateSolverVelocityIterationCountAttr().Set(
        config.tomato_solver_velocity_iterations
    )
    return True


def _apply_tomato_angular_damping(
    stage: object, tomato_prim_path: str, config: PhysicsTuningConfig
) -> bool:
    """release後にtray内で残留回転し続ける現象を抑えるため角速度減衰を設定する。

    settling判定（PlacementEvaluator）は接触impulseの有無に関わらず角速度そのものを
    毎stepチェックするため、接触ベースのねじり摩擦（torsional patch）が入力されない
    瞬間があっても効く剛体レベルのdampingを採用する
    （docs/reports/physics_levelup/step3-9_ci_release_flakiness_root_cause_analysis.md §12.4.2）。
    """
    if config.tomato_angular_damping <= 0.0:
        return False
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(tomato_prim_path)
    if not prim.IsValid():
        return False
    rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rigid_body_api.CreateAngularDampingAttr().Set(config.tomato_angular_damping)
    return True
