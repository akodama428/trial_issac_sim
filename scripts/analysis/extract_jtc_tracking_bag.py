"""rosbag(mcap)からJTC controller_stateとphaseをCSVへ展開する。

使い方 (コンテナ内、ROS環境をsource済みで):
    python3 scripts/analysis/extract_jtc_tracking_bag.py <bag_dir> <out_dir>
"""
import csv
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG_URI = sys.argv[1] if len(sys.argv) > 1 else "/tmp/freeze_bag"
OUT_DIR = sys.argv[2] if len(sys.argv) > 2 else "."

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG_URI, storage_id="mcap"),
    rosbag2_py.ConverterOptions("cdr", "cdr"),
)
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

state_rows = []
phase_rows = []
effort_rows = []
effort_joint_names = None
joint_names = None
while reader.has_next():
    topic, data, t_ns = reader.read_next()
    if topic == "/joint_trajectory_controller/controller_state":
        msg = deserialize_message(data, types[topic])
        if joint_names is None:
            joint_names = list(msg.joint_names)
        ref = getattr(msg, "reference", None) or getattr(msg, "desired", None)
        fb = getattr(msg, "feedback", None) or getattr(msg, "actual", None)
        state_rows.append(
            [t_ns * 1e-9] + list(ref.positions) + list(fb.positions)
        )
    elif topic == "/tomato_harvest/phase":
        msg = deserialize_message(data, types[topic])
        phase_rows.append([t_ns * 1e-9, msg.data])
    elif topic == "/isaac_joint_states":
        msg = deserialize_message(data, types[topic])
        if msg.effort and len(msg.effort) == len(msg.name):
            if effort_joint_names is None:
                effort_joint_names = list(msg.name)
            effort_rows.append([t_ns * 1e-9] + list(msg.effort))

with open(f"{OUT_DIR}/controller_state.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(
        ["t"] + [f"{n}_ref" for n in joint_names] + [f"{n}_fb" for n in joint_names]
    )
    writer.writerows(state_rows)
with open(f"{OUT_DIR}/phase.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "phase"])
    writer.writerows(phase_rows)
if effort_rows:
    with open(f"{OUT_DIR}/joint_effort.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t"] + effort_joint_names)
        writer.writerows(effort_rows)
print(f"joints={joint_names}")
print(
    f"state_rows={len(state_rows)} phase_rows={len(phase_rows)} "
    f"effort_rows={len(effort_rows)}"
)
