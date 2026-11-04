#!/usr/bin/env python3
"""Extract a DART rosbag into a pickle for offline analysis (run INSIDE the container).

    python3 tools/analysis/bag_extract.py /ws/bags/run_X /tmp/run_X.pkl

Stores: TF map->odom and odom->base_footprint as (stamp, x, y, yaw) lists,
static TFs, scans (stamp, receive time, angle_min, angle_inc, ranges, range_min),
/cmd_vel (receive time, v, w) and /rosout (stamp, node, text).
"""
import math
import pickle
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    bag, out = sys.argv[1], sys.argv[2]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    tf = {"map->odom": [], "odom->base_footprint": []}
    static, scans, cmds, logs = {}, [], [], []
    while r.has_next():
        topic, data, t_rx = r.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic in ("/tf", "/tf_static"):
            for tr in msg.transforms:
                k = f"{tr.header.frame_id}->{tr.child_frame_id}"
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                v = (st, tr.transform.translation.x, tr.transform.translation.y, yaw_q(tr.transform.rotation))
                if topic == "/tf_static":
                    static[k] = v
                elif k in tf:
                    tf[k].append(v)
        elif topic == "/scan":
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            scans.append((st, t_rx * 1e-9, msg.angle_min, msg.angle_increment, list(msg.ranges), msg.range_min))
        elif topic == "/cmd_vel":
            cmds.append((t_rx * 1e-9, msg.linear.x, msg.angular.z))
        elif topic == "/rosout":
            logs.append((msg.stamp.sec + msg.stamp.nanosec * 1e-9, msg.name, msg.msg))
    for k in tf:
        tf[k].sort()
    pickle.dump(dict(tf=tf, static=static, scans=scans, cmds=cmds, logs=logs), open(out, "wb"))
    print({k: len(v) for k, v in tf.items()}, "scans", len(scans), "cmds", len(cmds), "logs", len(logs))


if __name__ == "__main__":
    main()
