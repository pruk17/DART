#!/usr/bin/env python3
"""Convert a ROS occupancy-grid map (map.yaml + map.pgm) into a Gazebo .world.

Occupied cells become extruded wall boxes (horizontal run-length merged to keep
the box count low), so Gazebo's geometry matches the map AMCL localizes against.

Usage:
    python3 tools/map2world.py maps/map.yaml src/dart_ros/worlds/dart_room.world
Also prints the map bounds, a free-space centroid (good spawn point), and an
ASCII preview so delivery targets can be placed in free space.
"""
import sys
import yaml


def load_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    assert data[:2] == b"P5", "only binary PGM (P5) supported"
    # parse header tokens (magic, width, height, maxval), then binary pixels
    idx = 2
    toks = []
    while len(toks) < 3:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":  # comment line
            while idx < len(data) and data[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        toks.append(int(data[start:idx]))
    w, h, _maxval = toks
    idx += 1  # single whitespace after maxval
    px = data[idx:idx + w * h]
    return w, h, px


def main():
    yaml_path = sys.argv[1]
    out_path = sys.argv[2]
    import os
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    pgm = os.path.join(os.path.dirname(yaml_path), meta["image"])
    w, h, px = load_pgm(pgm)

    occ_pix = 100   # pixel < this (dark) -> occupied wall
    free_pix = 200  # pixel > this (light) -> free
    wall_h = 0.5

    def val(r, c):
        return px[r * w + c]

    # --- emit wall boxes (horizontal run-length merge per row) ---
    boxes = []
    for r in range(h):
        c = 0
        while c < w:
            if val(r, c) < occ_pix:
                c0 = c
                while c < w and val(r, c) < occ_pix:
                    c += 1
                length = c - c0
                xc = ox + (c0 + length / 2.0) * res
                yc = oy + (h - 1 - r + 0.5) * res
                boxes.append((xc, yc, length * res, res))
            else:
                c += 1

    # --- free-space centroid + bounds (for spawn / targets) ---
    fx = fy = 0.0
    nfree = 0
    for r in range(h):
        for c in range(w):
            if val(r, c) > free_pix:
                fx += ox + (c + 0.5) * res
                fy += oy + (h - 1 - r + 0.5) * res
                nfree += 1
    cx, cy = (fx / nfree, fy / nfree) if nfree else (ox, oy)

    models = []
    for i, (xc, yc, sx, sy) in enumerate(boxes):
        models.append(f"""    <model name="wall_{i}">
      <static>true</static>
      <pose>{xc:.3f} {yc:.3f} {wall_h/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {wall_h:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {wall_h:.3f}</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material></visual>
      </link>
    </model>""")

    world = f"""<?xml version="1.0"?>
<!-- Auto-generated from {meta['image']} by tools/map2world.py. Walls match the
     ROS map so Gazebo geometry == the map AMCL localizes against. -->
<sdf version="1.6">
  <world name="dart_room">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>false</shadows></scene>
{chr(10).join(models)}
  </world>
</sdf>
"""
    with open(out_path, "w") as f:
        f.write(world)

    xmin, xmax = ox, ox + w * res
    ymin, ymax = oy, oy + h * res
    print(f"map: {w}x{h} @ {res} m/cell")
    print(f"bounds: x[{xmin:.2f},{xmax:.2f}]  y[{ymin:.2f},{ymax:.2f}]")
    print(f"free-space centroid (spawn candidate): ({cx:.2f}, {cy:.2f})")
    print(f"walls: {len(boxes)} boxes -> {out_path}")
    print("ASCII (#=wall .=free (blank)=unknown):")
    for r in range(h):
        line = []
        for c in range(w):
            v = val(r, c)
            line.append("#" if v < occ_pix else ("." if v > free_pix else " "))
        print("".join(line))


if __name__ == "__main__":
    main()
