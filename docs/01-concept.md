# 1. Concept

## 1.1 What DART is

DART (*Decision-Adaptive Reward Targeting*) is an **online decision policy**
that tells a single robot **which task to do next** when it has many tasks and
**each task's value decays over time** — a perishable reward. It couples a
decaying-reward decision rule (borrowed from the MDP idea of discounting) with a
**reactive** navigation layer (a modified Bug2). It deliberately does **not**
use a global path planner such as A\*.

The idea is domain-agnostic. The application built here is a **food-delivery
robot** (a TurtleBot3 Burger serving tables); dishes cool down, melt or go
soggy at different rates. Other intended uses in the original concept: a
pick-and-place arm on a conveyor, or an AGV with delivery deadlines.

## 1.2 The problem

Classic dispatching picks the **nearest** task and assumes task value is
**constant**. When value decays, nearest-first lets valuable, fast-decaying
tasks expire while the robot serves cheap, slow ones. DART instead accounts
for the value that is actually collectable on arrival **and** for what the
rest of the tasks lose while the robot travels, and it **re-decides
continuously** as time passes.

The underlying combinatorial problem is a variant of the *Orienteering Problem
with time-decreasing profits*, solved online.

## 1.3 Notation

| Symbol | Meaning |
|---|---|
| `i` | candidate target |
| `v_i` | base value (importance) of target `i` |
| `λ_i` | decay rate of target `i`; higher = spoils faster |
| `t½,i` | half-life; `λ_i = ln 2 / t½,i` |
| `d_i` | straight-line distance robot → target `i` (no path planning) |
| `s` | robot nominal speed |
| `t_i = d_i / s` | estimated travel time |

Value after an elapsed time `τ` (exponential decay, i.e. a continuous-time
discount):

$$\text{value}_i(\tau) = v_i\,e^{-\lambda_i \tau}$$

## 1.4 The original decision rule (greedy Net score)

The concept defines a one-step score for each candidate:

$$\text{Net}_i = \underbrace{v_i e^{-\lambda_i t_i}}_{\text{gain}} \;-\; \underbrace{\sum_{j\neq i} v_j\left(1-e^{-\lambda_j t_i}\right)}_{\text{basket-bleed}}$$

and chooses `i* = argmax Net_i`.

- **Gain:** value of `i` left when the robot arrives.
- **Basket-bleed:** opportunity cost — how much every *other* target decays
  during the trip to `i`. This term stops the robot chasing a far target while
  many others spoil.
- Because `λ` appears in the score, fast-decaying tasks rank higher when
  distances are comparable (consistent with the *cμ-rule* of scheduling).

> **In the implemented system this greedy rule was replaced by a multi-leg
> lookahead planner** (see [Decision layer](03-decision.md)). The greedy rule
> remains in the code (`dart/scoring.py`, `dart/decider.py`) and in the
> headless simulator as a baseline.

## 1.5 The intended online loop

1. **Sense:** robot pose, remaining targets `{position, v, λ}`, LiDAR scan.
2. **Estimate times:** `t_i = d_i / s` for every target.
3. **Score** every feasible target.
4. **Select** the best; set it as the goal and define the **m-line**.
5. **Drive** along the m-line.
6. **Avoid:** if the LiDAR detects an obstacle, follow its boundary (Bug2),
   then rejoin the m-line. No map is needed for this.
7. **Re-decide continuously:** values decay as time passes; if another target
   beats the current one by a **switching margin ε** (hysteresis), abandon the
   current target, select the new one and **redraw the m-line**.
8. **Serve:** on reaching the selected target, do the task and remove it.
9. Repeat until no targets remain.

## 1.6 Navigation and localization (concept)

- **Path planning:** reactive, modified Bug2 instead of A\*.
- **Obstacle avoidance:** from the LiDAR every cycle; moving obstacles are
  simply sensed and avoided.
- **Localization:** a SLAM map is built **once**; **AMCL** localizes against
  it in real time.
- **Map usage:** the map is used **only** for localization and for goal
  coordinates — never for global path planning.

## 1.7 Standard vs DART-modified

**Bug2**

| Aspect | Standard Bug2 | DART-modified Bug2 |
|---|---|---|
| Goal | single fixed goal | the selected target, can change |
| m-line | one line start → goal | redrawn whenever the target changes |
| Leave rule | re-cross the m-line closer to the goal | same |
| Commitment | locked until the goal is reached | can abandon mid-travel if another target is better by ε |
| Decision driver | geometry only | geometry + time-decaying reward |

**MDP**

| Aspect | Standard MDP | DART use |
|---|---|---|
| Discount | per step `γ^k` | per real time `e^{-λt}` |
| Reward | fixed `R(s,a)` | decays over time `v e^{-λt}` |
| Model | transition model `P(s' \| s,a)` | not used |
| Solver | value/policy iteration, offline | online re-decision |

## 1.8 Origins of each ingredient

| Component | Origin |
|---|---|
| Bug / Bug2 | Lumelsky & Stepanov (1987), *Path-planning strategies for a point automaton*, Algorithmica |
| Bug path-length bounds | Choset et al. (2005), *Principles of Robot Motion*, MIT Press |
| Reward / discount | Bellman (1957), *A Markovian Decision Process* |
| Base problem | Orienteering Problem with decreasing / time-varying profits |
| Urgency ordering | cμ-rule (scheduling theory) |
| Localization | SLAM + AMCL (ROS 2 navigation stack) |
| Considered, rejected | Potential fields (Khatib 1986) — local minima |

**Claimed contributions** (as a *combination and application*, not a new
theory): `λ` as an explicit per-target variable in the reward; the
basket-bleed opportunity cost; value-aware continuous target switching inside
Bug2's m-line logic. In this implementation the lookahead planner generalises
the basket-bleed idea: decay of *all* targets along the *whole* remaining
route is simulated instead of a one-step penalty.

> Honesty note for any write-up: every ingredient exists in the literature;
> DART is a specific combination. No transition model is used, so the system
> should not be described as a full MDP.

## 1.9 Concept parameters

| Parameter | Meaning |
|---|---|
| `s` | nominal speed |
| `λ` per task type | from a half-life: `λ = ln 2 / t½` |
| `v` per task type | base value (importance, price) |
| `ε` | switching margin |
| `replan_rate` | how often step 7 runs |
