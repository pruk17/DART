#!/usr/bin/env python3
"""Replay one leg of an extracted rosbag through the current navigator (open loop).

    python3 tools/analysis/bag_replay.py /tmp/run_X.pkl --goal 0.03,-0.93 --from 40.3 \
        --map-config ros/dart_ros/config/real_small.yaml

Feeds the recorded poses and scans (from mission time --from) into
ReactiveNavigatorROS, prints mode changes, side choice, jumps of the tracked
boundary point and the agreement with the recorded /cmd_vel. Open loop: after
the first different decision the poses are still the recorded ones.
"""
import argparse
import bisect
import importlib.util
import math
import os
import pickle
import sys

import numpy as np
import yaml

from common import REPO, mission_start, poses_at_scans

spec = importlib.util.spec_from_file_location("nav", os.path.join(REPO, "ros/dart_ros/dart_ros/navigator.py"))
nav = importlib.util.module_from_spec(spec); sys.modules["nav"] = nav; spec.loader.exec_module(nav)


def load_robot(cfg_path):
    """robot: section with include: resolved (enough for this tool)."""
    raw = yaml.safe_load(open(cfg_path)) or {}
    base = {}
    if raw.get("include"):
        base = load_robot(os.path.join(os.path.dirname(cfg_path), raw["include"]))
    base.update(raw.get("robot", {}) or {})
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pkl"); ap.add_argument("--goal", required=True); ap.add_argument("--from", dest="t_from", type=float, required=True)
    ap.add_argument("--map-config", default=os.path.join(REPO, "ros/dart_ros/config/real_small.yaml"))
    a = ap.parse_args()
    r = load_robot(a.map_config)
    n = nav.ReactiveNavigatorROS(max_lin=r.get("speed", 0.2), max_ang=r.get("max_ang", 1.2), goal_tol=r.get("goal_tol", 0.15),
                                 safe_distance=r.get("safe_distance", 0.28), wall_offset=r.get("wall_offset", 0.2),
                                 wall_follow_speed=r.get("wall_follow_speed", 0.8))
    D = pickle.load(open(a.pkl, "rb")); t0 = mission_start(D); C = np.array(D["cmds"])
    goal = tuple(float(v) for v in a.goal.split(",")); started = False; prev = None; agree = []
    for sc, fp, _ in poses_at_scans(D):
        tm = sc[0] - t0
        if tm < a.t_from:
            continue
        if not started:
            n.set_goal(fp[:2], goal); started = True
        m0 = n.mode
        v, w = n.step(fp[:2], fp[2], list(sc[4]), sc[2], sc[3])
        j = min(bisect.bisect_left(C[:, 0], sc[1]), len(C) - 1)
        agree.append(abs(C[j, 1] - v) < 0.04 and abs(C[j, 2] - w) < 0.2)
        tp = n._track_pt
        tw = None if tp is None else (fp[0] + tp[0] * math.cos(fp[2]) - tp[1] * math.sin(fp[2]),
                                      fp[1] + tp[0] * math.sin(fp[2]) + tp[1] * math.cos(fp[2]))
        jump = tw is not None and prev is not None and math.dist(tw, prev) > 0.12
        if n.mode != m0 or jump:
            print(f"t={tm:6.1f} robot=({fp[0]:+.2f},{fp[1]:+.2f}) {n.mode.name} side={n._side} "
                  f"v={v:.2f} w={w:+.2f} track={None if tw is None else (round(tw[0], 2), round(tw[1], 2))}{'  JUMP' if jump else ''}")
        prev = tw
        if n.unreachable or n.at_goal(fp[:2]):
            print(f"t={tm:6.1f} {'UNREACHABLE' if n.unreachable else 'REACHED'}"); break
    print(f"agreement with recorded /cmd_vel: {100 * np.mean(agree):.0f}% over {len(agree)} scans")


if __name__ == "__main__":
    main()
