"""Shared helpers for the analysis scripts: map loading, pose reconstruction."""
import bisect
import math
import os
import sys

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))
from map2world import load_pgm  # noqa: E402


def load_map(map_yaml):
    """(occupied bool grid [row 0 = top], resolution, origin_x, origin_y)."""
    meta = yaml.safe_load(open(map_yaml))
    w, h, px = load_pgm(os.path.join(os.path.dirname(map_yaml), meta["image"]))
    occ = np.frombuffer(px, dtype=np.uint8).reshape(h, w) <= 50
    return occ, meta["resolution"], meta["origin"][0], meta["origin"][1]


def cell(x, y, occ, res, ox, oy):
    h = occ.shape[0]
    return h - 1 - np.floor((np.asarray(y) - oy) / res).astype(int), np.floor((np.asarray(x) - ox) / res).astype(int)


def compose(a, b):
    x, y, t = a
    bx, by, bt = b
    return (x + bx * math.cos(t) - by * math.sin(t), y + bx * math.sin(t) + by * math.cos(t), t + bt)


def interp(lst, t):
    """Pose (x, y, yaw) at time t from a sorted (stamp, x, y, yaw) list, or None."""
    ts = [v[0] for v in lst]
    i = bisect.bisect_right(ts, t) - 1
    if i < 0:
        return None
    if i + 1 < len(lst):
        a, b = lst[i], lst[i + 1]
        f = (t - a[0]) / max(1e-9, b[0] - a[0])
        d = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * d)
    return lst[i][1:]


def poses_at_scans(D):
    """Yield (scan_tuple, base_footprint pose, base_scan pose) in the map frame."""
    tf, st = D["tf"], D["static"]
    fp2s = compose(st["base_footprint->base_link"][1:], st["base_link->base_scan"][1:])
    mts = [v[0] for v in tf["map->odom"]]
    for sc in D["scans"]:
        i = bisect.bisect_right(mts, sc[0]) - 1
        ob = interp(tf["odom->base_footprint"], sc[0])
        if i < 0 or ob is None:
            continue
        fp = compose(tf["map->odom"][i][1:], ob)
        yield sc, fp, compose(fp, fp2s)


def mission_start(D):
    """Stamp of the first debug block (mission t = 0), or the first scan stamp."""
    for t, n, m in D["logs"]:
        if n == "dart_node" and "t=   0.0s" in m:
            return t
    return D["scans"][0][0]
