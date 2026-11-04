# Analysis tools

Used for the investigations in [docs/06](../../docs/06-development-history.md)
and [docs/07](../../docs/07-experiments.md). Run from the repository root.

| Script | Where | Purpose |
|---|---|---|
| `bag_extract.py` | inside the container (needs `rosbag2_py`) | rosbag → pickle (TF, scans, `/cmd_vel`, `/rosout`) |
| `bag_report.py` | numpy, scipy, matplotlib | rates, time in WALL_FOLLOW, distance to obstacles, events; PNG of obstacles + path (optionally per time window) |
| `bag_replay.py` | numpy, pyyaml | replay one leg through the current navigator, show decisions and agreement with recorded commands |
| `sim_harness.py` | numpy | kinematic simulator: real navigator on a real map with boxes, LDS-02 model, optional AMCL-like noise |
| `common.py` | — | map loading and pose reconstruction helpers |

```bash
python3 tools/analysis/bag_extract.py /ws/bags/run_X /tmp/run_X.pkl
python3 tools/analysis/bag_report.py /tmp/run_X.pkl --map maps/map_small.yaml --out run_X.png --windows 0,10,28,40,96
python3 tools/analysis/bag_replay.py /tmp/run_X.pkl --goal 0.03,-0.93 --from 40.3
python3 tools/analysis/sim_harness.py --map maps/map_small.yaml --start 0.97,-0.93 --goal 0.97,0.01 --box 0.97,-0.51,0.08
```
