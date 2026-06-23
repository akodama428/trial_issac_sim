from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.api.contracts import SceneSnapshot
from tomato_harvest_sim.simulator.scene_plan import ReviewScenePlan


@dataclass(frozen=True)
class SceneRuntimeDisplay:
    tomato_prim_path: str
    robot_tool_proxy_prim_path: str
    pregrasp_marker_prim_path: str
    tomato_driven_by_physics: bool


def build_scene_runtime_display(
    stage: object,
    plan: ReviewScenePlan,
    *,
    tomato_driven_by_physics: bool = False,
) -> SceneRuntimeDisplay:
    _add_robot_tool_proxy(stage, plan)
    _add_pregrasp_marker(stage, plan)
    return SceneRuntimeDisplay(
        tomato_prim_path=plan.tomato_prim_path,
        robot_tool_proxy_prim_path=plan.robot_tool_proxy_prim_path,
        pregrasp_marker_prim_path=plan.pregrasp_marker_prim_path,
        tomato_driven_by_physics=tomato_driven_by_physics,
    )


def sync_scene_runtime_display(stage: object, display: SceneRuntimeDisplay, snapshot: SceneSnapshot) -> None:
    if not display.tomato_driven_by_physics:
        _set_translate(stage, display.tomato_prim_path, snapshot.tomato_pose)
    _set_translate(stage, display.robot_tool_proxy_prim_path, snapshot.robot_tool_pose)
    if snapshot.pregrasp_pose is None:
        _set_visibility(stage, display.pregrasp_marker_prim_path, visible=False)
        return
    _set_translate(stage, display.pregrasp_marker_prim_path, snapshot.pregrasp_pose)
    _set_visibility(stage, display.pregrasp_marker_prim_path, visible=True)


def _add_robot_tool_proxy(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    proxy = UsdGeom.Sphere.Define(stage, plan.robot_tool_proxy_prim_path)
    proxy.AddTranslateOp().Set(
        Gf.Vec3d(plan.robot_tool_pose.x, plan.robot_tool_pose.y, plan.robot_tool_pose.z)
    )
    proxy.GetRadiusAttr().Set(0.018)
    proxy.CreateDisplayColorAttr([(0.15, 0.45, 0.95)])


def _add_pregrasp_marker(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    marker = UsdGeom.Cube.Define(stage, plan.pregrasp_marker_prim_path)
    initial_pose = plan.pregrasp_pose or plan.robot_tool_pose
    marker.AddTranslateOp().Set(Gf.Vec3d(initial_pose.x, initial_pose.y, initial_pose.z))
    marker.AddScaleOp().Set(Gf.Vec3f(0.012, 0.012, 0.012))
    marker.CreateDisplayColorAttr([(0.90, 0.18, 0.72)])
    if plan.pregrasp_pose is None:
        UsdGeom.Imageable(marker).MakeInvisible()


def _set_translate(stage: object, prim_path: str, pose: object) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    xformable = UsdGeom.Xformable(prim)
    translate_op = xformable.GetOrderedXformOps()[0]
    translate_op.Set(Gf.Vec3d(pose.x, pose.y, pose.z))


def _set_visibility(stage: object, prim_path: str, *, visible: bool) -> None:
    from pxr import UsdGeom

    imageable = UsdGeom.Imageable(stage.GetPrimAtPath(prim_path))
    if visible:
        imageable.MakeVisible()
        return
    imageable.MakeInvisible()
