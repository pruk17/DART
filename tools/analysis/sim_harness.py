#!/usr/bin/env python3
"""Kinematic simulator: the real Bug2 navigator on a real occupancy grid.

    python3 tools/analysis/sim_harness.py --map maps/map_small.yaml \
        --start 0.97,-0.93 --goal 0.97,0.01 --box 0.97,-0.51,0.08 [--noise 0.03 --seed 1]

Unicycle at 20 Hz; 360-beam LiDAR with LDS-02 behaviour (returns < 0.16 m
become 0); boxes (--box x,y,half or --rect x0,x1,y0,y1, repeatable) are added
to the grid; collision = an occupied cell within 0.105 m of the centre.
--noise adds an AMCL-like pose offset (random walk every 0.5 s) to the pose
the navigator sees, while the scan comes from the true pose.
Use run() from Python for parameter sweeps.
"""
import argparse
import importlib.util
import math
import os
import random
import sys

import numpy as np

from common import REPO, cell, load_map

spec = importlib.util.spec_from_file_location("nav", os.path.join(REPO, "ros/dart_ros/dart_ros/navigator.py"))
nav = importlib.util.module_from_spec(spec); sys.modules["nav"] = nav; spec.loader.exec_module(nav)

LDS_MIN = 0.16
ANG = np.linspace(-math.pi, math.pi, 360, endpoint=False)
STEPS = np.arange(0.02, 3.5, 0.01)


def make_grid(map_yaml, boxes=(), rects=()):
    occ, res, ox, oy = load_map(map_yaml)
    g = occ.copy()
    for (bx, by, half) in boxes:
        rects = list(rects) + [(bx - half, bx + half, by - half, by + half)]
    for (x0, x1, y0, y1) in rects:
        X, Y = np.meshgrid(np.arange(x0, x1 + 1e-9, res / 2), np.arange(y0, y1 + 1e-9, res / 2))
        r, c = cell(X, Y, g, res, ox, oy); g[r, c] = True
    return g, res, ox, oy


def scan(G, x, y, yaw):
    g, res, ox, oy = G
    a = yaw + ANG
    r, c = cell(x + np.outer(np.cos(a), STEPS), y + np.outer(np.sin(a), STEPS), g, res, ox, oy)
    hit = g[np.clip(r, 0, g.shape[0] - 1), np.clip(c, 0, g.shape[1] - 1)]
    idx = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    rng = np.where(idx >= 0, STEPS[np.maximum(idx, 0)], 0.0)
    rng[rng < LDS_MIN] = 0.0
    return rng.tolist()


def collides(G, x, y, rad=0.105):
    g, res, ox, oy = G
    t = np.linspace(0, 2 * math.pi, 24); rr = np.linspace(0, rad, 4)
    r, c = cell(x + np.outer(rr, np.cos(t)), y + np.outer(rr, np.sin(t)), g, res, ox, oy)
    return bool(g[r, c].any())


def run(G, start, goal, yaw=None, T=60.0, dt=0.05, noise=0.0, seed=0, speed=0.20, max_ang=1.2,
        safe=0.28, wall=0.20, wf_speed=0.8, goal_tol=0.15):
    """Returns (outcome, time, path)."""
    rnd = random.Random(seed); off = [0.0, 0.0, 0.0]
    n = nav.ReactiveNavigatorROS(max_lin=speed, max_ang=max_ang, goal_tol=goal_tol, safe_distance=safe,
                                 wall_offset=wall, wall_follow_speed=wf_speed)
    x, y = start
    yaw = math.atan2(goal[1] - y, goal[0] - x) if yaw is None else yaw
    n.set_goal((x, y), goal); t = 0.0; path = [(x, y)]
    while t < T:
        if noise and int(round(t / dt)) % 10 == 0:
            cap = 3 * noise
            off = [max(-cap, min(cap, off[0] + rnd.gauss(0, noise))), max(-cap, min(cap, off[1] + rnd.gauss(0, noise))),
                   max(-0.1, min(0.1, off[2] + rnd.gauss(0, 2.5 * noise)))]
        if n.at_goal((x + off[0], y + off[1])):
            return "REACHED", t, path
        if n.unreachable:
            return "UNREACHABLE", t, path
        v, w = n.step((x + off[0], y + off[1]), yaw + off[2], scan(G, x, y, yaw), -math.pi, 2 * math.pi / 360)
        nx, ny = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        if collides(G, nx, ny):
            return "COLLISION", t, path
        x, y, yaw = nx, ny, yaw + w * dt; t += dt; path.append((x, y))
    return "TIMEOUT", t, path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True); ap.add_argument("--start", required=True); ap.add_argument("--goal", required=True)
    ap.add_argument("--box", action="append", default=[]); ap.add_argument("--rect", action="append", default=[])
    ap.add_argument("--noise", type=float, default=0.0); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--safe", type=float, default=0.28); ap.add_argument("--wall", type=float, default=0.20)
    a = ap.parse_args()
    f = lambda s: tuple(float(v) for v in s.split(","))
    G = make_grid(a.map, [f(b) for b in a.box], [f(r) for r in a.rect])
    out, t, path = run(G, f(a.start), f(a.goal), noise=a.noise, seed=a.seed, safe=a.safe, wall=a.wall)
    print(f"{out} after {t:.1f} s, path {sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)):.2f} m")


if __name__ == "__main__":
    main()
