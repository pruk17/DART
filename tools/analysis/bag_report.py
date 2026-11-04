#!/usr/bin/env python3
"""Report on an extracted DART rosbag: obstacles, trajectory, modes, rates.

    python3 tools/analysis/bag_report.py /tmp/run_X.pkl --map maps/map_small.yaml --out run_X.png [--windows 0,10,28,40,96]

Prints data/command rates, time in WALL_FOLLOW, stop share while following,
distance to the nearest return while following, and the node's events.
The PNG shows the map, obstacle returns (> 10 cm from any mapped wall) and the
trajectory (orange = WALL_FOLLOW). With --windows, one panel per mission-time
window (obstacles moved during a run become visible).
"""
import argparse
import bisect
import math
import pickle
import re

import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import cell, load_map, mission_start, poses_at_scans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pkl")
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", default="bag_report.png")
    ap.add_argument("--windows", default="", help="mission-time edges, e.g. 0,10,28,40,96")
    a = ap.parse_args()
    D = pickle.load(open(a.pkl, "rb"))
    occ, res, ox, oy = load_map(a.map)
    dt = distance_transform_edt(~occ) * res
    t0 = mission_start(D)
    blocks = []
    for t, n, m in D["logs"]:
        mm = re.search(r"mode=(\w+)", m)
        if n == "dart_node" and mm:
            blocks.append((t, mm.group(1)))
    bt = [b[0] for b in blocks]

    def mode_at(t):
        i = bisect.bisect_right(bt, t) - 1
        return blocks[i][1] if i >= 0 else "?"

    rows = []
    for sc, fp, sp in poses_at_scans(D):
        R = np.array(sc[4]); A = sc[2] + sc[3] * np.arange(len(R)); ok = (R > 0) & np.isfinite(R)
        X = sp[0] + R[ok] * np.cos(sp[2] + A[ok]); Y = sp[1] + R[ok] * np.sin(sp[2] + A[ok])
        r, c = cell(X, Y, occ, res, ox, oy)
        inside = (r >= 0) & (r < occ.shape[0]) & (c >= 0) & (c < occ.shape[1])
        d = np.full(X.shape, 9.0); d[inside] = dt[r[inside], c[inside]]
        rows.append(dict(t=sc[0] - t0, fp=fp, obst=np.c_[X[d > 0.10], Y[d > 0.10]],
                         nearest=R[ok].min() if ok.any() else np.inf, mode=mode_at(sc[0])))
    C = np.array(D["cmds"])
    mv = C[(abs(C[:, 1]) > 1e-3) | (abs(C[:, 2]) > 1e-3)]
    if len(mv):
        W = C[(C[:, 0] >= mv[0, 0]) & (C[:, 0] <= mv[-1, 0])]
        wf = [c for c in W if mode_at(c[0]) == "WALL_FOLLOW"]
        print(f"cmd rate {len(W) / (mv[-1, 0] - mv[0, 0]):.1f}/s (one DART publisher = 20)"
              + (f", stop share while following {100 * np.mean([c[1] == 0 for c in wf]):.0f}%" if wf else ""))
    hug = np.array([x["nearest"] for x in rows if x["mode"] == "WALL_FOLLOW"])
    if len(hug):
        print(f"WALL_FOLLOW ~{len(hug) / max(1, len(rows)) * (rows[-1]['t'] - rows[0]['t']):.0f}s; nearest return "
              f"median {np.median(hug):.3f} m, p10 {np.percentile(hug, 10):.3f}, max {hug.max():.3f}")
    for t, n, m in D["logs"]:
        if n == "dart_node" and "╔" not in m:
            print(f"  t={t - t0:6.1f}  {m.strip()[:100]}")

    edges = [float(x) for x in a.windows.split(",")] if a.windows else [-1e9, 1e9]
    k = len(edges) - 1
    fig, axs = plt.subplots(1, k, figsize=(6 * k, 6), squeeze=False)
    ext = [ox, ox + occ.shape[1] * res, oy, oy + occ.shape[0] * res]
    for j in range(k):
        ax = axs[0][j]; sel = [x for x in rows if edges[j] <= x["t"] < edges[j + 1]]
        ax.imshow(np.where(occ, 0, 255), cmap="gray", origin="upper", extent=ext, vmin=0, vmax=255, alpha=0.5)
        if sel:
            O = np.vstack([x["obst"] for x in sel]); ax.scatter(O[:, 0], O[:, 1], s=1, c="r", alpha=0.05)
            P = np.array([x["fp"][:2] for x in sel]); m = np.array([x["mode"] == "WALL_FOLLOW" for x in sel])
            ax.plot(P[:, 0], P[:, 1], "b-", lw=1); ax.scatter(P[m, 0], P[m, 1], s=4, c="orange")
            ax.plot(P[0, 0], P[0, 1], "g^", ms=9)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title("all" if k == 1 else f"t {edges[j]:.0f}-{edges[j + 1]:.0f} s", fontsize=9)
    fig.tight_layout(); fig.savefig(a.out, dpi=85); print("wrote", a.out)


if __name__ == "__main__":
    main()
