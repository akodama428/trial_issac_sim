from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import Pose3D, SceneSnapshot
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.config import (
    MoveItPlannerConfig,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.geometry import (
    quaternion_from_pose,
)

_PANDA_FINGER_LINKS = ("panda_leftfinger", "panda_rightfinger")


@dataclass(frozen=True)
class TomatoPlanningSceneOps:
    add_world_tomato: bool
    remove_world_tomato: bool
    add_attached_tomato: bool
    remove_attached_tomato: bool
    add_world_stem: bool
    remove_world_stem: bool


def attached_tomato_touch_links(
    end_effector_link: str,
) -> tuple[str, ...]:
    """把持物との接触を許可するgripper linkを返す。

    attached objectは固定先linkとの接触だけが暗黙に許可される。実際に
    tomatoを挟む左右fingerもtouch linkへ含め、把持直後の意図的な接触を
    MoveItのstart-state collisionとして扱わないようにする。
    """
    return tuple(dict.fromkeys((end_effector_link, *_PANDA_FINGER_LINKS)))


def tomato_planning_scene_ops(
    *,
    attach_tomato: bool,
    planning_scene_has_attached_tomato: bool,
) -> TomatoPlanningSceneOps:
    if attach_tomato:
        if planning_scene_has_attached_tomato:
            # DETACHINGでattachしたobjectをMOVING_TO_PLACEでも維持する。
            # 同じADD/REMOVEを再送するとPlanningScene適用が失敗し得るため、
            # 既に完了したworld→attached遷移は繰り返さない。
            return TomatoPlanningSceneOps(
                add_world_tomato=False,
                remove_world_tomato=False,
                add_attached_tomato=False,
                remove_attached_tomato=False,
                add_world_stem=False,
                remove_world_stem=False,
            )
        # 把持後は同じtomatoをworld objectとattached objectの両方へ残さない。
        # AttachedCollisionObject.ADDが同じIDのworld objectをattached側へ
        # 自動的に移管するため、world側の明示REMOVEは同時に送らない。
        # stemはgripperとの意図的接触中なので、pull計画の開始状態から除外する。
        return TomatoPlanningSceneOps(
            add_world_tomato=False,
            remove_world_tomato=False,
            add_attached_tomato=True,
            remove_attached_tomato=False,
            add_world_stem=False,
            remove_world_stem=True,
        )
    return TomatoPlanningSceneOps(
        add_world_tomato=True,
        remove_world_tomato=False,
        add_attached_tomato=False,
        remove_attached_tomato=planning_scene_has_attached_tomato,
        add_world_stem=True,
        remove_world_stem=False,
    )


class PlanningSceneManager:
    """Own the world/attached tomato transition state."""

    def __init__(self, config: MoveItPlannerConfig) -> None:
        self._config = config
        self._has_attached_tomato = False

    def apply(
        self,
        *,
        clients: object,
        scene_snapshot: SceneSnapshot,
        base_frame_id: str,
        attach_tomato: bool,
    ) -> bool:
        request = build_planning_scene_request(
            config=self._config,
            scene_snapshot=scene_snapshot,
            base_frame_id=base_frame_id,
            tomato_ops=tomato_planning_scene_ops(
                attach_tomato=attach_tomato,
                planning_scene_has_attached_tomato=self._has_attached_tomato,
            ),
        )
        if not clients.apply_planning_scene(
            request, timeout_sec=self._config.planning_timeout_sec
        ):
            return False
        self._has_attached_tomato = attach_tomato
        return True


def build_planning_scene_request(
    *,
    config: MoveItPlannerConfig,
    scene_snapshot: SceneSnapshot,
    base_frame_id: str,
    tomato_ops: TomatoPlanningSceneOps,
) -> object:
    from moveit_msgs.msg import (
        AttachedCollisionObject,
        CollisionObject,
        PlanningScene,
        RobotState,
    )
    from moveit_msgs.srv import ApplyPlanningScene

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state = RobotState()
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [
        _box_collision_object(
            object_id="tomato_branch",
            frame_id=base_frame_id,
            pose=scene_snapshot.branch_pose,
            size_xyz=config.branch_size_m,
        ),
        *_tray_collision_objects(
            frame_id=base_frame_id,
            tray_pose=scene_snapshot.tray_pose,
            tray_inner_size_m=config.tray_inner_size_m,
            tray_wall_thickness_m=config.tray_wall_thickness_m,
            collision_margin_m=config.tray_collision_margin_m,
        ),
    ]
    if tomato_ops.add_world_stem:
        scene.world.collision_objects.append(
            _box_collision_object(
                object_id="tomato_stem",
                frame_id=base_frame_id,
                pose=scene_snapshot.stem_pose,
                size_xyz=config.stem_size_m,
            )
        )
    if tomato_ops.remove_world_stem:
        scene.world.collision_objects.append(
            _remove_collision_object(
                object_id="tomato_stem", frame_id=base_frame_id
            )
        )
    if tomato_ops.add_world_tomato:
        scene.world.collision_objects.append(
            _sphere_collision_object(
                object_id="target_tomato",
                frame_id=base_frame_id,
                pose=scene_snapshot.tomato_pose,
                radius_m=config.attached_tomato_radius_m,
            )
        )
    if tomato_ops.remove_world_tomato:
        scene.world.collision_objects.append(
            _remove_collision_object(
                object_id="target_tomato", frame_id=base_frame_id
            )
        )

    attached_objects: list[object] = []
    if tomato_ops.add_attached_tomato:
        attached = AttachedCollisionObject()
        attached.link_name = config.end_effector_link
        attached.object = _sphere_collision_object(
            object_id="target_tomato",
            frame_id=config.end_effector_link,
            pose=Pose3D(*config.attached_tomato_offset_m, 0.0, 0.0, 0.0),
            radius_m=config.attached_tomato_radius_m,
        )
        attached.touch_links = list(
            attached_tomato_touch_links(config.end_effector_link)
        )
        attached_objects.append(attached)
    if tomato_ops.remove_attached_tomato:
        attached = AttachedCollisionObject()
        attached.link_name = config.end_effector_link
        attached.object = CollisionObject()
        attached.object.id = "target_tomato"
        attached.object.header.frame_id = config.end_effector_link
        attached.object.operation = CollisionObject.REMOVE
        attached_objects.append(attached)
    scene.robot_state.attached_collision_objects = attached_objects

    request = ApplyPlanningScene.Request()
    request.scene = scene
    return request


def _tray_collision_objects(
    *,
    frame_id: str,
    tray_pose: Pose3D,
    tray_inner_size_m: tuple[float, float, float],
    tray_wall_thickness_m: float,
    collision_margin_m: float,
) -> tuple[object, ...]:
    inner_x, inner_y, inner_z = tray_inner_size_m
    wall, margin = tray_wall_thickness_m, collision_margin_m
    half_z, wall_height = inner_z / 2.0, inner_z + wall
    specs = (
        (
            "place_tray_base",
            Pose3D(tray_pose.x, tray_pose.y, tray_pose.z, 0, 0, 0),
            (
                inner_x + 2 * wall + 2 * margin,
                inner_y + 2 * wall + 2 * margin,
                wall + 2 * margin,
            ),
        ),
        (
            "place_tray_wall_front",
            Pose3D(
                tray_pose.x + inner_x / 2 + wall / 2,
                tray_pose.y,
                tray_pose.z + half_z + margin / 2,
                0,
                0,
                0,
            ),
            (
                wall + 2 * margin,
                inner_y + 2 * wall + 2 * margin,
                wall_height + margin,
            ),
        ),
        (
            "place_tray_wall_back",
            Pose3D(
                tray_pose.x - inner_x / 2 - wall / 2,
                tray_pose.y,
                tray_pose.z + half_z + margin / 2,
                0,
                0,
                0,
            ),
            (
                wall + 2 * margin,
                inner_y + 2 * wall + 2 * margin,
                wall_height + margin,
            ),
        ),
        (
            "place_tray_wall_left",
            Pose3D(
                tray_pose.x,
                tray_pose.y + inner_y / 2 + wall / 2,
                tray_pose.z + half_z + margin / 2,
                0,
                0,
                0,
            ),
            (
                inner_x + 2 * margin,
                wall + 2 * margin,
                wall_height + margin,
            ),
        ),
        (
            "place_tray_wall_right",
            Pose3D(
                tray_pose.x,
                tray_pose.y - inner_y / 2 - wall / 2,
                tray_pose.z + half_z + margin / 2,
                0,
                0,
                0,
            ),
            (
                inner_x + 2 * margin,
                wall + 2 * margin,
                wall_height + margin,
            ),
        ),
    )
    return tuple(
        _box_collision_object(
            object_id=object_id,
            frame_id=frame_id,
            pose=pose,
            size_xyz=size,
        )
        for object_id, pose, size in specs
    )


def _box_collision_object(
    *,
    object_id: str,
    frame_id: str,
    pose: Pose3D,
    size_xyz: tuple[float, float, float],
) -> object:
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in size_xyz]
    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [_pose_msg_from_pose(pose)]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _sphere_collision_object(
    *,
    object_id: str,
    frame_id: str,
    pose: Pose3D,
    radius_m: float,
) -> object:
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [float(radius_m)]
    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [_pose_msg_from_pose(pose)]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _remove_collision_object(*, object_id: str, frame_id: str) -> object:
    from moveit_msgs.msg import CollisionObject

    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.operation = CollisionObject.REMOVE
    return collision_object


def _pose_msg_from_pose(pose: Pose3D) -> object:
    from geometry_msgs.msg import Pose

    message = Pose()
    message.position.x = float(pose.x)
    message.position.y = float(pose.y)
    message.position.z = float(pose.z)
    message.orientation = quaternion_from_pose(pose)
    return message
