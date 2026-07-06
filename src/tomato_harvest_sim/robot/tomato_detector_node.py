"""tomato_detector_node — カメラ画像と TF を入力にトマト位置を推定し target_estimate を publish する。

アーキテクチャ仕様: ROS2_COMPONENT_ARCHITECTURE.md §tomato_detector_node
"""
from __future__ import annotations


def main() -> None:
    import json
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    from tomato_harvest_sim.msg.contracts import (
        CameraFrame,
        JointStateSnapshot,
        Pose3D,
        TfTreeSnapshot,
    )
    from tomato_harvest_sim.msg.topics import (
        FIXED_CAMERA_TOPIC,
        TARGET_ESTIMATE_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import target_estimate_to_json
    from tomato_harvest_sim.robot.perception import TomatoTargetEstimator

    rclpy.init()

    class TomatoDetectorNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("tomato_detector_node")
            self._estimator = TomatoTargetEstimator()
            self._last_camera_pose: Pose3D | None = None
            self._last_target_pose: Pose3D | None = None

            self._pub = self.create_publisher(String, TARGET_ESTIMATE_TOPIC, 10)
            self.create_subscription(Image, FIXED_CAMERA_TOPIC, self._on_image, 10)
            self.create_subscription(TFMessage, "/tf", self._on_tf, 10)

        def _on_image(self, _msg: Image) -> None:
            self._try_estimate()

        def _on_tf(self, msg: TFMessage) -> None:
            for transform in msg.transforms:
                child = getattr(transform, "child_frame_id", "")
                if child == "target_tomato_frame":
                    t = transform.transform.translation
                    self._last_target_pose = Pose3D(
                        x=float(t.x), y=float(t.y), z=float(t.z),
                        roll=0.0, pitch=0.0, yaw=0.0,
                    )
                elif child == "fixed_camera_frame":
                    t = transform.transform.translation
                    self._last_camera_pose = Pose3D(
                        x=float(t.x), y=float(t.y), z=float(t.z),
                        roll=0.0, pitch=0.0, yaw=0.0,
                    )
            self._try_estimate()

        def _try_estimate(self) -> None:
            if self._last_camera_pose is None or self._last_target_pose is None:
                return
            camera_frame = CameraFrame(
                camera_name="fixed_camera",
                topic_name=FIXED_CAMERA_TOPIC,
                frame_id="fixed_camera_frame",
                camera_pose=self._last_camera_pose,
                target_world_pose=self._last_target_pose,
            )
            tf_tree = TfTreeSnapshot(
                robot_base_frame_id="panda_link0",
                camera_frame_id="fixed_camera_frame",
                target_frame_id="target_tomato_frame",
                robot_base_pose=Pose3D(0, 0, 0, 0, 0, 0),
                camera_pose=self._last_camera_pose,
                target_pose=self._last_target_pose,
            )
            estimate = self._estimator.estimate(camera_frame, tf_tree)
            if estimate is None:
                return
            out = String()
            out.data = target_estimate_to_json(estimate)
            self._pub.publish(out)

    node = TomatoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
