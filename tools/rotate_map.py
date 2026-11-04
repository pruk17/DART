#!/usr/bin/env python3
"""Rotate a ROS occupancy-grid map (map.yaml + .pgm) about the map origin.

Writes a NEW map whose frame is the old one rotated by `deg` (counter-
clockwise), so walls that are tilted by -deg in the old map come out
axis-aligned. The point (0, 0) stays where it was; a pose (x, y, yaw) in the
old frame becomes (x*c - y*s, x*s + y*c, yaw + deg) with c, s = cos/sin(deg).

Usage:
    python3 tools/rotate_map.py maps/map_test.yaml maps/map_test_0deg 11.5
"""
import math
import os
import sys

import numpy as np
import yaml

from map2world import load_pgm


def main():
    in_yaml, out_base, deg = sys.argv[1], sys.argv[2], float(sys.argv[3])
    meta = yaml.safe_load(open(in_yaml))
    w, h, px = load_pgm(os.path.join(os.path.dirname(in_yaml), meta["image"]))
    img = np.frombuffer(px, dtype=np.uint8).reshape(h, w)
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)

    # cell centres of the old map (row 0 = top) and where they land in the new frame
    rows, cols = np.mgrid[0:h, 0:w]
    x = ox + (cols + 0.5) * res
    y = oy + (h - 1 - rows + 0.5) * res
    u, v = x * c - y * s, x * s + y * c
    u0, v0 = np.floor(u.min() / res) * res, np.floor(v.min() / res) * res
    W = int(np.ceil((u.max() - u0) / res)) + 1
    H = int(np.ceil((v.max() - v0) / res)) + 1

    # new cell centres -> old map coordinates (inverse rotation), nearest cell
    nr, nc = np.mgrid[0:H, 0:W]
    nu = u0 + (nc + 0.5) * res
    nv = v0 + (H - 1 - nr + 0.5) * res
    bx, by = nu * c + nv * s, -nu * s + nv * c
    oc = np.floor((bx - ox) / res).astype(int)
    orow = h - 1 - np.floor((by - oy) / res).astype(int)
    inside = (oc >= 0) & (oc < w) & (orow >= 0) & (orow < h)
    out = np.full((H, W), 205, dtype=np.uint8)            # unknown outside the old map
    out[inside] = img[orow[inside], oc[inside]]

    # nearest-neighbour resampling can open 1-cell holes in thin walls: also mark a
    # cell occupied if any old occupied cell centre lands within ~0.7 cell of it
    occ = img <= 50
    ou = ((u[occ] - u0) / res).astype(int)
    ov = H - 1 - ((v[occ] - v0) / res).astype(int)
    ok = (ou >= 0) & (ou < W) & (ov >= 0) & (ov < H)
    out[ov[ok], ou[ok]] = 0

    pgm = out_base + ".pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(out.tobytes())
    meta_out = dict(meta)
    meta_out["image"] = os.path.basename(pgm)
    meta_out["origin"] = [round(float(u0), 4), round(float(v0), 4), 0.0]
    with open(out_base + ".yaml", "w") as f:
        yaml.safe_dump(meta_out, f, default_flow_style=None, sort_keys=False)
    print(f"wrote {pgm} ({W}x{H}) and {out_base}.yaml, origin {meta_out['origin']}, rotated {deg:+.1f} deg")


if __name__ == "__main__":
    main()
