"""Render an end-of-run map: the robot's path over the occupancy map.

Called by dart_node when a run completes. Draws (if available) the ROS map the
robot localized on, the trajectory coloured by time, the start pose, and every
delivery point with its visit order, food type and remaining quality (or an X
if it expired). Saved as a PNG next to the run log.
"""

from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")  # headless: no display needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


def _load_map(map_yaml: str):
    """Return (image array, extent) for a ROS map.yaml + P5 PGM, or None."""
    if not map_yaml or not os.path.exists(map_yaml):
        return None
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    pgm = os.path.join(os.path.dirname(map_yaml), meta["image"])
    with open(pgm, "rb") as f:
        data = f.read()
    if data[:2] != b"P5":
        return None
    # header: magic, width, height, maxval (skip comments), then raw bytes
    idx, toks = 2, []
    while len(toks) < 3:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        toks.append(int(data[start:idx]))
    w, h, _ = toks
    img = np.frombuffer(data[idx + 1: idx + 1 + w * h], dtype=np.uint8).reshape(h, w)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    return img, (ox, ox + w * res, oy, oy + h * res)


def render_path_map(
    out_png: str,
    path: list[tuple[float, float]],
    targets_pos: dict[str, tuple[float, float]],
    target_types: dict[str, str],
    delivered: dict[str, tuple[int, float]],
    expired_ids: list[str],
    title: str,
    map_yaml: str = "",
    unreachable_ids: list[str] | None = None,
) -> bool:
    """Draw and save the path map. Returns True on success."""
    fig, ax = plt.subplots(figsize=(7, 7), dpi=120)

    m = _load_map(map_yaml)
    if m is not None:
        img, extent = m
        ax.imshow(img, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255)

    # trajectory, coloured from start (dark) to end (bright)
    if len(path) >= 2:
        xs = np.array([p[0] for p in path])
        ys = np.array([p[1] for p in path])
        ax.plot(xs, ys, color="#1f77b4", linewidth=1.2, alpha=0.5, zorder=2)
        ax.scatter(xs, ys, c=np.arange(len(xs)), cmap="viridis", s=6, zorder=3)
        ax.plot(xs[0], ys[0], marker="^", color="#2ca02c", markersize=13,
                markeredgecolor="k", zorder=5)
        ax.annotate("START", (xs[0], ys[0]), textcoords="offset points", xytext=(8, -14),
                    fontsize=9, color="#2ca02c", weight="bold")

    # delivery points
    for tid, (x, y) in targets_pos.items():
        ftype = target_types.get(tid, "")
        name = f"{tid} ({ftype})" if ftype else tid
        if tid in delivered:
            order, pct = delivered[tid]
            ax.plot(x, y, "o", color="#f2c014", markersize=16, markeredgecolor="k", zorder=4)
            ax.text(x, y, str(order), ha="center", va="center", fontsize=9, weight="bold", zorder=6)
            label = f"{name}\n#{order} · {pct:.0f}%"
        elif tid in expired_ids:
            ax.plot(x, y, "X", color="#d62728", markersize=14, markeredgecolor="k", zorder=4)
            label = f"{name}\nexpired"
        elif unreachable_ids and tid in unreachable_ids:
            ax.plot(x, y, "X", color="#7f3fbf", markersize=14, markeredgecolor="k", zorder=4)
            label = f"{name}\nunreachable"
        else:
            ax.plot(x, y, "o", color="#bbbbbb", markersize=12, markeredgecolor="k", zorder=4)
            label = f"{name}\nnot reached"
        ax.annotate(label, (x, y), textcoords="offset points", xytext=(10, 6), fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8), zorder=7)

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True
