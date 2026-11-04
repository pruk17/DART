# 3. Decision layer

The **decision** is *which target to drive to next*. Everything about *how* to
get there is the navigation layer ([document 4](04-navigation.md)), which does
not consider value or time. The only information the navigation layer passes
back to the decision is "this target is unreachable".

## 3.1 Value model and expiry (`dart/decay.py`)

Every cycle the node decays each remaining target by the real elapsed time:

$$v_i \leftarrow v_i\,e^{-\lambda_i \Delta t}$$

`Target.v` therefore always holds the **current** value; the starting values
are snapshotted once (`snapshot_initial_values`). A target is **expired** and
dropped when $v_i < 0.05\,v_i^0$ (`expire_threshold`). Decay keeps running
while the robot stops to serve, while it waits for stale data and while it
goes round obstacles.

### Choosing values and half-lives

Food types were derived from the 60 s mission deadline: decide what fraction
of value each dish should still have if it is served exactly at 60 s, then

$$t_{1/2} = \frac{60 \ln 2}{\ln(1/q_{60})}$$

| type | v | t½ | value left at 60 s |
|---|---:|---:|---:|
| ice_cream | 4 | 30 s | 25 % |
| fries | 3 | 35 s | 30 % |
| hot_soup | 4 | 40 s | 35 % |
| main_dish | 5 | 60 s | 50 % |
| cold_drink | 2 | 95 s | 65 % |
| salad | 3 | 185 s | 80 % |

These are design choices for the demonstration, not measured food data. `v`
expresses importance or price.

## 3.2 Why the greedy rule was replaced

The concept's greedy score (document 1) evaluates one leg at a time. In the
headless simulator it collected more total value than nearest-first but did so
by **letting more targets expire** — it "sacrificed" low-value tasks because it
never evaluated what happens over the rest of the route, and nothing in it
judges whether a target is genuinely too decayed to be worth serving. The
requirement became: *"if I go to A, then B, then C …, which order
gives the largest total? Simulate the decay over all legs and choose — do not
sacrifice anything unnecessarily."* That is a lookahead over complete orders.

## 3.3 Lookahead planner (`dart/planner.py`)

**Objective.** For an order $(k_1,\dots,k_n)$ starting from the robot position
$p_0$ with speed $s$ and service time $t_s$:

$$t_{k_1} = \frac{\lVert p_{k_1}-p_0\rVert}{s},\qquad
t_{k_j} = t_{k_{j-1}} + t_s + \frac{\lVert p_{k_j}-p_{k_{j-1}}\rVert}{s}$$

$$J = \sum_{j=1}^{n} \begin{cases} v_{k_j} e^{-\lambda_{k_j} t_{k_j}} & \text{if } \ge 0.05\,v^0_{k_j}\\ 0 & \text{otherwise}\end{cases}$$

`simulate_sequence()` walks this clock exactly.

**Search.**

| Targets | Method | Property |
|---|---|---|
| N ≤ 7 | `plan_full_permutation`: every order (7! = 5040) | exact optimum of the model |
| N > 7 | `plan_two_opt`: nearest-neighbour start, then 2-opt segment reversals while J improves (≤ 20 passes) | fast, may stop at a local optimum |

An earlier version used 2-opt for all sizes; twice it returned a visibly worse
order (a local optimum), which is why small instances are solved exactly.

The plan is recomputed **every cycle** (20 Hz) from the current position and
current values, so it reacts to decay and to the robot's actual progress.

**Model assumptions.** Travel time uses **straight-line distance**; the
planner does not know about obstacles or detours. This is the main limitation
of the decision layer (section 3.7).

## 3.4 Commitment and switching (`dart_node`)

The robot drives to one **current** target. Each cycle:

1. If there is no current target (start, or just delivered), take the first
   target of the best plan.
2. Otherwise compute `best = J(plan)` and `keep = J(current first, then the
   plan's order of the rest)`. **Switch** only if

$$J_{\text{plan}} > (1+\varepsilon)\,J_{\text{keep}},\qquad \varepsilon = 0.15$$

The margin prevents flip-flopping between two nearly equal choices, which would
reset Bug2 repeatedly. A switch is allowed **at any time, also while following
an obstacle boundary** (concept step 7 / table 8.1); the navigator then draws a
new m-line from the current position. An earlier version only switched while
driving straight; it was changed to follow the concept after a run where the
planner preferred another target for about a minute during a long detour.

## 3.5 Delivery rules

| Event | Action |
|---|---|
| Within `goal_tol` (0.15 m) of the current target | deliver: log `reached`, add value, drop target, stop for `service_time` (3 s); the next target starts from this spot with a new m-line |
| Within `goal_tol` of **another** target | deliver it **on the way** (`reached X on the way to Y`), stop 3 s, then restart the current target from this spot (new m-line, Bug2 state cleared) |
| Navigator reports unreachable | log `unreachable`, drop target, choose again |
| No targets left | write run log and path map |

Concept step 8 only serves the *selected* target. Delivering on the way widens
the serving condition without changing the decision: food is on the robot, so
passing a table is an opportunity to serve it. It costs one service stop for
the current target and saves a later return trip.

The restart after an on-the-way delivery matters: an earlier version kept the
old m-line and hit point, and in one run the robot then circled an obstacle for
about 40 s because the old m-line was far away and never crossed again
(see [history](06-development-history.md#deliver-on-the-way)).

## 3.6 Worked example: why the robot passed a target without serving it

Run with targets m3 (hot_soup) and m4 (fries) left; the current target m3 was
behind an obstacle and the detour passed right by m4. Values from the debug
log, totals computed with the real planner:

| t (s) | dist m4 | dist m3 (straight) | plan first | J(plan) / J(keep m3) | switch at ε=15 % | at 6 % |
|---:|---:|---:|---|---:|---|---|
| 66.7 | 0.89 m | 1.38 m | m4 | +2.1 % | no | no |
| 77.0 | 0.44 m | 1.07 m | m4 | +3.2 % | no | no |
| 79.1 | 0.10 m | 0.96 m | m4 | +5.4 % | no | no |
| 80.1 | 0.04 m | 0.96 m | m4 | **+5.9 %** | no | no |
| 89.4 | 0.43 m | 0.53 m | m3 | 0 % | – | – |

The planner believed m3 was only ~5 s away in a straight line, so serving m3
first looked only ~6 % worse. In reality m3 was reached at 92 s and the robot
had to come back for m4 at 104 s. Lowering ε would not have helped unless
≤ 5 %, at the cost of more switching elsewhere. Delivering on the way solves
this case directly (m4 would have been served at ~79 s).

## 3.7 Limitations of the decision layer

- Straight-line travel times underestimate the cost of targets behind
  obstacles; the decision does not learn from an ongoing detour.
- Service time and speed are constants; real speed during boundary following
  is 80 % and slower in corners.
- Hysteresis is a fixed fraction; with low remaining values the absolute
  difference that ε protects against becomes small.
