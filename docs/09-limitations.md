# 9. Limitations and future work

## Decision layer

| Limitation | Effect | Possible direction |
|---|---|---|
| Travel time = straight-line distance / speed | targets behind obstacles look cheap; the robot keeps a target during long detours | inflate the estimate for the current target while in `WALL_FOLLOW` (time already spent, distance along the boundary), or a coarse occupancy-based distance for estimates only (keeps the reactive navigation) |
| Fixed relative hysteresis ε = 0.15 | with low remaining values, worthwhile switches are blocked | absolute + relative margin, or margin scaled by remaining value |
| Constant speed and service time in the model | optimistic plans in cluttered rooms | use measured average speed |
| Full permutation up to 7 targets | 8+ targets fall back to 2-opt (local optima) | Or-opt / restarts, or dynamic programming over subsets for ≤ 12 |

## Navigation layer

| Limitation | Effect | Possible direction |
|---|---|---|
| Targets in a pocket whose mouth is ≈ 0.4 m | may be judged unreachable | attempted (contour-length test), rejected for complexity; revisit with better geometry |
| No general loop detection | following an island that never crosses the m-line nor returns to the hit point can circle for long | detect revisiting a point of the current wall-follow path and restart Bug2 from there (new m-line), give up after two loops |
| LDS-02 blind below 0.16 m | obstacle placed against the robot may be invisible | remember close returns with odometry; bumper |
| Hug distance floor 0.20 m | cannot pass gaps < 0.30 m that the robot could physically fit | smaller `body_clear` with a better footprint model (rectangle, LiDAR offset) |
| Moving obstacles | treated as static each cycle; a moving obstacle can drag the boundary follower | track obstacle motion, wait briefly when blocked |

## Localization and system

| Limitation | Possible direction |
|---|---|
| Reliable `/tf` over Wi-Fi stalls 1–2 s on packet loss | better Wi-Fi, DDS tuning, or running AMCL on the robot |
| Battery drops crash `turtlebot3_node` | monitor `/battery_state`, stop before low voltage |
| Small, nearly square room makes global localization ambiguous | asymmetric landmarks; always give a start pose |

## Evaluation still to do

- Repeated trials per scenario (≥ 5) with fixed obstacle placements, to report
  means and spread.
- Comparison against a baseline on the robot (nearest-first, greedy Net) with
  the same targets.
- Sensitivity of quality to ε, `wall_offset`, speed and half-lives.
