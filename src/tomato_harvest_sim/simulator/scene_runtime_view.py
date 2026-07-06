from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import SceneSnapshot
from tomato_harvest_sim.simulator.debug_visualization import SceneRuntimeDebugState
from tomato_harvest_sim.simulator.scene_plan import ReviewScenePlan


@dataclass(frozen=True)
class SceneRuntimeDisplay:
    tomato_prim_path: str
    robot_tool_proxy_prim_path: str
    pregrasp_marker_prim_path: str
    grasp_marker_prim_path: str
    pull_marker_prim_path: str
    place_marker_prim_path: str
    target_estimate_marker_prim_path: str
    active_target_marker_prim_path: str
    active_waypoint_marker_prim_path: str
    perception_ray_prim_path: str
    pregrasp_path_prim_path: str
    grasp_path_prim_path: str
    pull_path_prim_path: str
    place_path_prim_path: str
    tracking_path_prim_path: str
    tomato_driven_by_physics: bool


def build_scene_runtime_display(
    stage: object,
    plan: ReviewScenePlan,
    *,
    tomato_driven_by_physics: bool = False,
) -> SceneRuntimeDisplay:
    _add_robot_tool_proxy(stage, plan)
    _add_debug_root(stage)
    _add_pregrasp_marker(stage, plan)
    _add_sphere_marker(stage, "/World/Debug/Planner/GraspMarker", color_rgb=(0.95, 0.32, 0.18), radius_m=0.013)
    _add_sphere_marker(stage, "/World/Debug/Planner/PullMarker", color_rgb=(0.20, 0.84, 0.92), radius_m=0.013)
    _add_sphere_marker(stage, "/World/Debug/Planner/PlaceMarker", color_rgb=(0.22, 0.78, 0.40), radius_m=0.013)
    _add_sphere_marker(stage, "/World/Debug/Perception/TargetEstimateMarker", color_rgb=(0.98, 0.78, 0.14), radius_m=0.014)
    _add_sphere_marker(stage, "/World/Debug/Tracking/ActiveTargetMarker", color_rgb=(0.98, 0.98, 0.98), radius_m=0.013)
    _add_sphere_marker(stage, "/World/Debug/Tracking/ActiveWaypointMarker", color_rgb=(0.50, 0.88, 1.00), radius_m=0.011)
    _add_polyline(stage, "/World/Debug/Perception/EstimateRay", color_rgb=(0.98, 0.78, 0.14), width=0.004)
    _add_polyline(stage, "/World/Debug/Planner/PregraspPath", color_rgb=(0.90, 0.18, 0.72), width=0.005)
    _add_polyline(stage, "/World/Debug/Planner/GraspPath", color_rgb=(0.95, 0.32, 0.18), width=0.005)
    _add_polyline(stage, "/World/Debug/Planner/PullPath", color_rgb=(0.20, 0.84, 0.92), width=0.005)
    _add_polyline(stage, "/World/Debug/Planner/PlacePath", color_rgb=(0.22, 0.78, 0.40), width=0.005)
    _add_polyline(stage, "/World/Debug/Tracking/ActivePath", color_rgb=(0.98, 0.98, 0.98), width=0.007)
    return SceneRuntimeDisplay(
        tomato_prim_path=plan.tomato_prim_path,
        robot_tool_proxy_prim_path=plan.robot_tool_proxy_prim_path,
        pregrasp_marker_prim_path=plan.pregrasp_marker_prim_path,
        grasp_marker_prim_path="/World/Debug/Planner/GraspMarker",
        pull_marker_prim_path="/World/Debug/Planner/PullMarker",
        place_marker_prim_path="/World/Debug/Planner/PlaceMarker",
        target_estimate_marker_prim_path="/World/Debug/Perception/TargetEstimateMarker",
        active_target_marker_prim_path="/World/Debug/Tracking/ActiveTargetMarker",
        active_waypoint_marker_prim_path="/World/Debug/Tracking/ActiveWaypointMarker",
        perception_ray_prim_path="/World/Debug/Perception/EstimateRay",
        pregrasp_path_prim_path="/World/Debug/Planner/PregraspPath",
        grasp_path_prim_path="/World/Debug/Planner/GraspPath",
        pull_path_prim_path="/World/Debug/Planner/PullPath",
        place_path_prim_path="/World/Debug/Planner/PlacePath",
        tracking_path_prim_path="/World/Debug/Tracking/ActivePath",
        tomato_driven_by_physics=tomato_driven_by_physics,
    )


def sync_scene_runtime_display(
    stage: object,
    display: SceneRuntimeDisplay,
    snapshot: SceneSnapshot,
    *,
    debug_state: SceneRuntimeDebugState | None = None,
) -> None:
    if not display.tomato_driven_by_physics:
        _set_translate(stage, display.tomato_prim_path, snapshot.tomato_pose)
    _set_translate(stage, display.robot_tool_proxy_prim_path, snapshot.robot_tool_pose)
    effective_debug_state = debug_state or SceneRuntimeDebugState()
    _sync_marker(stage, display.pregrasp_marker_prim_path, effective_debug_state.pregrasp_pose)
    _sync_marker(stage, display.grasp_marker_prim_path, effective_debug_state.grasp_pose)
    _sync_marker(stage, display.pull_marker_prim_path, effective_debug_state.pull_pose)
    _sync_marker(stage, display.place_marker_prim_path, effective_debug_state.place_pose)
    _sync_marker(stage, display.target_estimate_marker_prim_path, effective_debug_state.target_estimate_pose)
    _sync_marker(stage, display.active_target_marker_prim_path, effective_debug_state.active_target_pose)
    _sync_marker(stage, display.active_waypoint_marker_prim_path, effective_debug_state.active_waypoint_pose)
    _sync_polyline(stage, display.perception_ray_prim_path, effective_debug_state.perception_ray_points)
    _sync_polyline(stage, display.pregrasp_path_prim_path, effective_debug_state.pregrasp_path_points)
    _sync_polyline(stage, display.grasp_path_prim_path, effective_debug_state.grasp_path_points)
    _sync_polyline(stage, display.pull_path_prim_path, effective_debug_state.pull_path_points)
    _sync_polyline(stage, display.place_path_prim_path, effective_debug_state.place_path_points)
    _sync_polyline(stage, display.tracking_path_prim_path, effective_debug_state.tracking_path_points)


def _add_debug_root(stage: object) -> None:
    from pxr import UsdGeom

    UsdGeom.Xform.Define(stage, "/World/Debug")
    UsdGeom.Xform.Define(stage, "/World/Debug/Perception")
    UsdGeom.Xform.Define(stage, "/World/Debug/Planner")
    UsdGeom.Xform.Define(stage, "/World/Debug/Tracking")


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


def _add_sphere_marker(stage: object, prim_path: str, *, color_rgb: tuple[float, float, float], radius_m: float) -> None:
    from pxr import Gf, UsdGeom

    marker = UsdGeom.Sphere.Define(stage, prim_path)
    marker.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    marker.GetRadiusAttr().Set(radius_m)
    marker.CreateDisplayColorAttr([color_rgb])
    UsdGeom.Imageable(marker).MakeInvisible()


def _add_polyline(stage: object, prim_path: str, *, color_rgb: tuple[float, float, float], width: float) -> None:
    from pxr import Gf, UsdGeom

    curve = UsdGeom.BasisCurves.Define(stage, prim_path)
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateCurveVertexCountsAttr([2])
    curve.CreatePointsAttr([Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0)])
    curve.CreateWidthsAttr([width])
    curve.CreateDisplayColorAttr([color_rgb])
    UsdGeom.Imageable(curve).MakeInvisible()


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


def _sync_marker(stage: object, prim_path: str, pose: object | None) -> None:
    if pose is None:
        _set_visibility(stage, prim_path, visible=False)
        return
    _set_translate(stage, prim_path, pose)
    _set_visibility(stage, prim_path, visible=True)


def _sync_polyline(stage: object, prim_path: str, points: tuple[object, ...]) -> None:
    from pxr import Gf, UsdGeom

    if len(points) < 2:
        _set_visibility(stage, prim_path, visible=False)
        return
    curve = UsdGeom.BasisCurves(stage.GetPrimAtPath(prim_path))
    curve.GetCurveVertexCountsAttr().Set([len(points)])
    curve.GetPointsAttr().Set([Gf.Vec3f(point.x, point.y, point.z) for point in points])
    _set_visibility(stage, prim_path, visible=True)
