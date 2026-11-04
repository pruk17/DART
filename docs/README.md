# DART documentation

This folder explains the whole system — what it is meant to do, how every part
works, why it was built that way, how it was tested, and what is still open —
in enough depth to understand it without access to the development machine and
to write a report from it. The top-level [README](../README.md) is the short
version.

## Reading order

| # | Document | What it answers |
|---|---|---|
| 1 | [Concept](01-concept.md) | The problem, the original DART idea, notation, the intended online loop, the origin of each idea |
| 2 | [Architecture](02-architecture.md) | Hardware/software split, ROS graph, packages and files, the 20 Hz cycle step by step, configuration loading |
| 3 | [Decision layer](03-decision.md) | Value model, lookahead planner, switching with hysteresis, delivery rules, worked examples |
| 4 | [Navigation layer](04-navigation.md) | Textbook Bug2, the DART modification, every rule of the implemented boundary follower and why it exists |
| 5 | [Localization and safety](05-localization-safety.md) | SLAM map, AMCL tuning, pose smoothing, stale-data stop, shutdown, communication pitfalls |
| 6 | [Development history](06-development-history.md) | Chronological record of problems found on the robot and in simulation, their evidence and the fix |
| 7 | [Experiments and results](07-experiments.md) | Test methods (robot runs, rosbag replay, kinematic simulator) and all measured results |
| 8 | [Operations guide](08-operations.md) | Running, configuring targets and food types, recording and analysing runs, troubleshooting |
| 9 | [Limitations and future work](09-limitations.md) | Known weaknesses and how they could be addressed |

Analysis scripts used in documents 6 and 7 are in [`tools/analysis/`](../tools/analysis/).

## Glossary

| Term | Meaning |
|---|---|
| Target, task, table | A delivery point with a position, a base value `v` and a decay rate `λ` |
| Half-life `t½` | Time for a value to halve; `λ = ln 2 / t½` |
| Quality | Value at delivery as a percentage of the base value |
| Plan / order | Visiting sequence chosen by the lookahead planner |
| Current target | The one target the robot is driving to (committed) |
| ε (epsilon) | Hysteresis margin for switching the current target |
| Bug2 | Reactive navigation: drive to the goal, follow an obstacle's boundary when blocked, leave on the m-line |
| m-line | Straight line from where a goal was set to the goal |
| Hit point | Where boundary following started |
| Leave | Returning from boundary following to driving at the goal |
| `wall_offset` | Distance kept from the boundary while following it |
| Swept corridor | The strip the robot's body would cover driving straight ahead |
| AMCL | Adaptive Monte Carlo Localization (particle filter on a known map) |
| Stale data | Pose or scan that stopped updating |
| Run log / path map | Per-run text summary and PNG trajectory written by the node |
