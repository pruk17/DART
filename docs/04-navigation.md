# 4. Navigation layer (DART-modified Bug2)

Implementation: `ros/dart_ros/dart_ros/navigator.py`, class
`ReactiveNavigatorROS`. Input each cycle: pose `(x, y, yaw)` in the map frame
and the raw LiDAR scan. Output: `(v, w)`. Two modes: `GO_TO_GOAL` and
`WALL_FOLLOW`.

## 4.1 Textbook Bug2

```
while not at goal:
    move towards the goal along the start-goal line (m-line)
    if an obstacle is hit:
        remember the hit point
        follow the obstacle's boundary (always the same direction, left or right)
        until the m-line is met again closer to the goal than the hit point
              and the robot is able to move towards the goal
        if the robot returns to the hit point: the goal is unreachable
```

Bug2 is greedy (it takes the first opportunity that looks better) and is
complete for a point robot. The DART modification (document 1, table 1.7):
the goal can change at any time and the m-line is then redrawn; the leave rule
itself is unchanged.

## 4.2 From a point robot to a real one

The textbook robot is a point with perfect contact sensing. The real robot has
a 0.105 m radius, a LiDAR 3.2 cm behind its centre that cannot see closer than
0.16 m, noisy localization and finite turning rate. Every extra rule below
exists to translate "follow the boundary" into something that works under
these conditions; the reason for each is given in the table and the full story
is in [Development history](06-development-history.md).

## 4.3 Parameters

| Name | Real robot (`real_small`) | `common.yaml` / code default | Meaning |
|---|---:|---:|---|
| `safe_distance` | 0.28 m | 0.35 | obstacle this close in the goal direction = hit |
| `wall_offset` | 0.20 m | 0.30 | distance held from the boundary |
| `body_clear` | 0.19 m | 0.19 | depth of the swept corridor (LiDAR frame) |
| `half_width` | 0.14 m | 0.14 | half-width of the swept corridor (radius 0.105 + 3.5 cm) |
| `lookahead` | 0.15 m | 0.15 | pure-pursuit carrot distance along the boundary |
| `track_gate` | 0.15 m | 0.15 | max jump of the tracked boundary point per cycle |
| `front_arc_deg` | 30° | 30° | sector width for the hit test |
| `leave_margin` | 0.05 m | 0.05 | must be this much closer than at the hit |
| `hit_depart_radius` / `hit_return_radius` | 0.30 / 0.15 m | | unreachable test |
| `follow_range` | 1.5 m | 1.5 | boundary points considered |
| `wall_follow_speed` | 0.8 | 0.8 | speed fraction while following |
| `max_lin` / `max_ang` | 0.20 m/s / 1.2 rad/s | | velocity caps |

Why `wall_offset` cannot go below ~0.20 m: it must stay above `body_clear`
(0.19 m), otherwise the boundary being followed would itself trip the corridor
guard; and `body_clear` cannot shrink much because the LDS-02 is blind below
0.16 m and the robot's nose is 0.137 m in front of the LiDAR. At 0.20 m the
side gap between body and obstacle is ≈ 9.5 cm.

## 4.4 GO_TO_GOAL

- Heading error $e_\theta$ to the goal; $w = \text{clip}(1.5\,e_\theta)$,
  $v = v_{max}\cos e_\theta$ (0 when facing away, so it turns in place first).
- **Swept-corridor guard:** if any scan point lies less than 0.19 m ahead and
  within ±0.14 m of the centre line, `v = 0` (turning is still allowed: a round
  robot cannot hit anything by rotating).

### Hit test (→ WALL_FOLLOW)

Either of:

1. **Goal blocked:** the nearest return within ±15° of the **goal
   direction** is closer than `safe_distance` **and** closer than the goal.
   Using the goal direction (not the current heading) means a wall the robot
   faces while turning, or a wall behind the goal, is not a "hit".
2. **Body blocked toward the goal:** heading within 60° of the goal and the
   swept corridor is occupied — this catches obstacles inside the LiDAR's
   blind range straight ahead, which test 1 cannot see.

On a hit the navigator stores the hit point and its distance to the goal, the
signed side of the m-line, the blocking point as the first **tracked boundary
point**, and **chooses the side** to go round.

### Side choice

Bug2 allows going round either way. The navigator compares the clearance in
two 80° sectors centred 70° left and right of the goal direction and goes
round on the **more open side** (equal within 5 cm → default "right", i.e.
obstacle kept on the right). Going round the left keeps the obstacle on the
right. The choice is fixed until the next hit.

## 4.5 WALL_FOLLOW

### Boundary tracking (`_tracked_point`)

Bug2 follows the boundary of **the obstacle that was hit**, not whatever is
nearest. The navigator keeps one boundary point in the **robot frame** and,
each cycle, takes the nearest scan point within `track_gate` (0.15 m) of the
previous one. The point moves less than 2 cm per cycle, so this works from the
scan alone and is immune to localization jumps.

- Points more than 120° behind the robot are ignored (boundary already
  passed).
- **Merging:** a point from *another* obstacle joins the boundary only if it
  is closer than `body_clear`, lies ahead (|bearing| ≤ 90°), **and** the gap
  between it and the tracked boundary is narrower than the robot can pass
  (`2·half_width + 2 cm` = 0.30 m). Then the two really are one obstacle for
  the robot. A wider gap stays open.
- If nothing is within the gate (contact lost), re-acquire the nearest point
  on the obstacle side, else the nearest point.

### Swept corridor while following

If the corridor (0.19 m × ±0.14 m) contains points:

- **across the path or on the obstacle side** → it is the next face of the
  boundary (a concave corner): track it from now on and **turn in place away
  from the obstacle side**;
- **wholly on the free side** → it is something else across a passable gap
  (e.g. a room wall): keep the boundary and turn in place away from it, i.e.
  toward the boundary.

### Pure-pursuit boundary following

With the tracked point $P$ (range $r$), the unit normal from $P$ to the robot
$\hat n$, and the tangent $\hat t$ along the boundary (obstacle kept on side
$s$, $\hat t = (-s\,n_y,\; s\,n_x)$):

$$C = P + d\,\hat n + L\,\hat t,\qquad d = \texttt{wall\_offset},\; L = \texttt{lookahead}$$

$C$ (the carrot) lies on the offset path a little ahead. With
$\alpha = \operatorname{atan2}(C_y, C_x)$:

- if $|\alpha| > 60°$: turn in place, $w = \text{clip}(1.5\,\alpha)$;
- else $v = 0.8\,v_{max}\cos\alpha$, $w = 2 v \sin\alpha / \lVert C\rVert$.

Along a straight wall $C$ is straight ahead. Round a convex corner $P$ stays at
the corner while $\hat n$ rotates, so $C$ swings round it and the robot turns
on the `wall_offset` circle instead of drifting outward. Being too far or too
close simply moves $C$ inward or outward. This replaced a two-term
proportional controller whose heading lag (≈ v/(r·k) ≈ 24°) made the robot
drift 0.1–0.2 m outward at every corner and stop-and-turn about half the time.

### Leave (→ GO_TO_GOAL)

All three textbook conditions:

1. the signed distance to the m-line changed sign since the last cycle
   (the m-line is the infinite line through start and goal);
2. the robot is at least `leave_margin` closer to the goal than at the hit;
3. the goal direction is not blocked (hit test 1 is false).

### Unreachable

After moving more than 0.30 m away from the hit point, coming back within
0.15 m of it means the robot went all the way round: `unreachable` is set and
the decision layer skips the target.

## 4.6 State reset

`set_goal(start, goal)` (new target, switch, after any delivery) draws a new
m-line and clears the mode, hit point, tracked point and side choice.

## 4.7 Behaviour summary

| Situation | Behaviour |
|---|---|
| Obstacle on the straight path, open on one side | goes round on the open side at ~0.20 m, leaves on the m-line |
| Obstacle touching a wall | treated as one obstacle with the wall; goes round the free end |
| Two obstacles with a gap ≥ 0.30 m | passes through the gap |
| Gap < 0.30 m | treated as closed |
| Goal enclosed | reports unreachable after one lap |
| Goal behind the robot | turns in place, then drives |
| Something placed against the robot (< 0.16 m) | may be invisible to the LiDAR |
