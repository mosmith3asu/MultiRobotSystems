# Multi-Robot Oil Spill Perimeter Tracking (MATLAB)

A MATLAB simulation of a six-robot swarm that finds a spreading oil spill and then spreads out along its edge to track it. It runs in two stages:

1. **Flocking to the spill.** The robots start as a group in the lower-left corner. They fly toward the spill's highest-concentration point, using velocity alignment and a cohesion potential between neighbors to stay together. The stage ends as soon as any robot senses the concentration threshold.
2. **Tracking the edge with potential fields.** Each robot then follows its own potential field. An attractive term pulls it onto the iso-concentration contour *c = c<sub>thr</sub>* (the spill "perimeter"). Repulsive terms spread the robots apart, and a "plateau" term steers any robot that has lost the concentration gradient toward a teammate who still has it.

Everything runs on a 100 × 100 grid over the unit square, with a spill that keeps diffusing while the robots move.

This folder is part of the `MultiRobotSystems` coursework repo. The sibling folders `Flocking/` and `GillespiesDirectMethod/` hold the earlier homework: digraph consensus, flocking ODEs, and stochastic chemical-reaction models.

---

## Quick start

```matlab
cd OilSpill
Main          % runs flocking, then 2000 potential-field iterations, and opens all figures
```

* **Requirements:** base MATLAB. `ode45`, `contour`, `pcolor` and `mink` are all built in. `SimpleSpillEstimator.m` also needs the Curve Fitting Toolbox (`fit`, `poly44`), but it is turned off by default.
* **Runtime:** 30–40 minutes for the full run in GNU Octave. MATLAB's JIT should be much faster. Most of the time goes to the cell-by-cell diffusion loop and to printing to the console on every iteration.
* **Randomness:** starting positions and velocities are random, and no seed is set. Add `rng(<seed>)` at the top of `Main.m` to make a run repeatable.

## File overview

| File | Role |
|---|---|
| `Main.m` | Entry script. Sets the spill and gain parameters, builds the `Enviorment` and `Functions` objects, and calls `Flock()`, then `Pot()`. |
| `Enviorment.m` | Spill model (a `handle` class): `initialize(n)`, `step(n)`, `current_map()`, `sample_concentration(loc)`, `max_conc()`. |
| `Flock.m` | Stage 1. Sets the starting states, integrates `Final_FlockingODE` with `ode45` over t ∈ [0, 100], and plots the paths and pairwise distances. |
| `Final_FlockingODE.m` | Right-hand side of the second-order flocking model for 6 robots on a fixed neighbor graph, with a goal-seeking term and a stop when the threshold is sensed. |
| `Pot.m` | Stage 2. The potential-field control loop (spill step → forces → motion update), plus logging and all analysis plots. |
| `Functions.m` | Library class: attractive, Cartesian-repulsive, radial-repulsive and plateau potentials; the kinematic update; polar-coordinate helpers; plotting helpers. |
| `SimpleSpillEstimator.m` | Experimental map estimator (a `poly44` surface fit to what the robots sampled). Turned off: `plot_est = 0`, and its calls are commented out. |

## Model details

### Spill environment (`Enviorment.m`)
* **Seeding:** six seed cells near the grid center, (50,50), (45,45), (55,55), (50,55), (55,50) and (50,45), each start with amplitude `pt_amp = 50`.
* **Diffusion:** a discrete 4-neighbor diffusion step. Every cell above 1e-4 gives `diff_rate × c` (0.05) to each of its four neighbors and keeps `c(1 − 4·diff_rate)`. The loop updates the grid in place, so mass can spread across several cells in one sweep.
* **Warm-up and edges:** `initialize(50)` pre-diffuses the spill before the robots are released. Cells on the grid edge lose mass without passing it on, so the total amount of oil slowly shrinks.
* **Perimeter:** the spill edge is the contour `conc_thresh = 0.15`.

### Stage 1 – Flocking (`Flock.m`, `Final_FlockingODE.m`)
* **State and start:** the state is *z = [x; y; v<sub>x</sub>; v<sub>y</sub>]* for N = 6 robots. Starting x values are `linspace(0.05, 0.3)`, starting y values are `U(0, 0.2)`, and starting velocities are `U(0, 0.2)`.
* **Goal term:** ẋ<sub>i</sub> = v<sub>i</sub> + 10·(p<sub>goal</sub> − p<sub>i</sub>), where p<sub>goal</sub> is the location of the spill's maximum concentration.
* **Neighbor term:** v̇<sub>i</sub> = −Σ<sub>j∈N(i)</sub> (v<sub>i</sub> − v<sub>j</sub>) − Σ<sub>j∈N(i)</sub> V′(r<sub>ij</sub>)·(p<sub>i</sub> − p<sub>j</sub>)/r<sub>ij</sub>. The cohesion potential is V(r) = ln²r + 1/r, with distances scaled ×6 before it is applied.
* **Neighbor graph (undirected, 7 edges):** 1–2, 1–3, 2–4, 3–5, 4–5, 4–6, 5–6.
* **Stop condition:** once any robot samples c ≥ c<sub>thr</sub>, the derivative is set to zero, which freezes the flock. The final positions and velocities are passed to stage 2.

### Stage 2 – Potential-field perimeter control (`Pot.m`, `Functions.agent_potentials`)
Each iteration: the spill advances one step, then

**F<sub>i</sub> = 15·F<sub>att</sub> + 5·F<sub>rep</sub> + 12·F<sub>plateau</sub>**

| Term | Definition |
|---|---|
| **Attractive** | F<sub>att</sub> = −∇\|c<sub>thr</sub> − c(x,y)\|, sampled at the robot's cell. This pulls each robot onto the threshold contour. |
| **Plateau** (`neighoring_conc_pot`) | Applies only when the robot's own concentration is below 1e-4, i.e. it has no gradient to follow. The robot is pulled with magnitude 1/r toward the teammate whose sampled concentration is closest to c<sub>thr</sub>. |
| **Repulsive** | Robots in plateau mode use a *radial* repulsion in polar coordinates around the spill peak (eq. 16 of the Ocean Engineering 2020 paper cited in the code, [doi:10.1016/j.oceaneng.2020.107238](https://doi.org/10.1016/j.oceaneng.2020.107238)). All other robots use an inverse-square *Cartesian* repulsion (Howard, Matarić & Sukhatme, *Mobile Sensor Network Deployment using Potential Fields*, 2002). |
| **Motion update** | v ← v + F, with each velocity component capped at ±0.5 cells/iteration; then p ← p + v, clamped to the grid. |

### Key parameters (`Main.m` / `Functions.m`)

| Parameter | Value | Parameter | Value |
|---|---|---|---|
| `N_agents` | 6 | `T` (stage-2 iterations) | 2000 |
| `conc_thresh` | 0.15 | `n_initial` (warm-up steps) | 50 |
| `diffusion_rate` | 0.05 | `attractive_amp` (`pt_amp`) | 50 |
| gains `a_scale / r_scale / p_scale` | 15 / 5 / 12 | `max_vel` | 0.5 cells/it |
| flocking goal gain | 10 | flocking distance scale | 6 |

---

## Results

The figures below come from running the unmodified algorithm in **GNU Octave 8.4** with random seeds 1, 2 and 3. A few small syntax changes were needed for Octave; see [Reproducibility notes](#reproducibility-notes). The figures are from seed 1 unless noted. Distances in stage 2 are in grid cells, where 1 cell = 0.01 of the domain width.

### Stage 1 – The flock reaches the spill

![Flocking paths](figures/01_flocking_paths.png)

*The six robots (black ✱ = start) move together toward the spill peak (blue ✱). The flock freezes when the first robot reaches the 0.15 contour.*

![Pairwise distances while flocking](figures/02_flocking_distances.png)

*Distances along the seven graph edges shrink steadily as the flock closes in. They go flat at t ≈ 0.1, when the stop condition fires. Everything after that is the frozen state.*

### Stage 2 – Robots spread around the spill edge

| Start of stage 2 | After 2000 iterations |
|---|---|
| ![Initial](figures/03_spill_agents_initial.png) | ![Final](figures/05_spill_agents_final.png) |

*The white line is the c = 0.15 perimeter and black **+** marks are the robots. At the start, the robots sit in a cluster at the lower-left edge of a young, concentrated spill (peak ≈ 1.0). By the end they are spread around the perimeter of a much wider and weaker spill (peak ≈ 0.18).*

![Perimeter tracks](figures/04_perimeter_tracks.png)

*Black contours show the perimeter at iterations 1, 667 and 1334, and the colored lines are robot tracks. Most robots lock onto the edge within a few hundred iterations and then move back and forth along it. A robot that starts off the spill takes a wide plateau-mode loop before it finds the gradient.*

![Perimeter tracks, seed 2](figures/04b_perimeter_tracks_seed2.png)

*Seed 2 shows the plateau term's worst case. One robot (orange) circles far outside the spill for about 700 iterations before it is pulled back to the perimeter.*

| Concentration sampled by each robot | Distance to the two nearest neighbors |
|---|---|
| ![Concentrations](figures/06_agent_concentrations.png) | ![Neighbor distances](figures/07_neighbor_distances.png) |

*Robots are sampled every 20 iterations, so the x-axis runs 0–100 samples, or 0–2000 iterations. Every robot converges to the 0.15 threshold and stays within about ±0.01 of it. Neighbor spacing jumps after the flock breaks up, then settles at about 10–15 cells as the robots spread around the edge.*

### Summary across seeds

| Metric | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| Extra spill steps during flocking (ODE RHS calls) | 245 | 318 | 205 |
| Iterations until **all** robots sample ≥ 0.5·c<sub>thr</sub> | ≈ 240 | ≈ 740 | ≈ 580 |
| Mean sampled concentration, iterations 1000–2000 (target 0.15) | 0.144 ± 0.009 | 0.144 ± 0.010 | 0.145 ± 0.009 |
| Mean \|c − c<sub>thr</sub>\|, iterations 1000–2000 | 0.0083 | 0.0088 | 0.0080 |
| Mean nearest-neighbor distance, iterations 1000–2000 (cells) | 12.3 | 12.1 | 12.5 |
| Minimum separation to either of the two nearest neighbors, iterations 1000–2000 (cells) | 6.5 | 8.1 | 8.8 |
| Final perimeter mean radius (cells) | 9.5 | 8.7 | 9.9 |
| Final robot distance to the perimeter, mean / max (cells) | 1.4 / 2.6 | 3.2 / 4.9 | 1.6 / 5.1 |
| Largest angular gap between robots around the spill (6 evenly spaced = 60°) | 95° | 69° | 76° |
| Final peak concentration | 0.177 | 0.172 | 0.180 |

**Takeaways**
* **Tracking accuracy:** in all three runs, the robots' sampled concentration settles about 4% below the threshold, and every robot ends within about 5 cells of the true perimeter.
* **No collisions:** in iterations 1000–2000, no robot came closer than 6.5 cells to either of its two nearest neighbors, in any run.
* **Spacing:** the robots cover the whole edge, but the spacing is uneven. The largest gap is 69–95°, compared with 60° for perfect spacing.
* **Slowest step:** getting every robot onto the spill takes the longest. It depends on how the flock breaks up, and the plateau term can send a robot on a long detour (seed 2).
* **Run length:** by iteration 2000, the spill's peak has dropped to just above the threshold. A much longer run would lose the perimeter entirely, because the edge cells drain mass (see below).

---

## Known issues and limitations

These are behaviors in the current code worth knowing before reusing it:

1. **`spill_centers` is ignored.** `Main.m` draws random centers (`N_spills = 3`), but `Enviorment.initialize` replaces them with six hard-coded seed cells.
2. **The spill moves during the ODE solve.** `Final_FlockingODE` calls `spill.step(1)` on every right-hand-side evaluation. How far the spill advances in stage 1 therefore depends on the solver (205–318 steps above), not on simulated time.
3. **`unique(z,'rows')` sorts the flocking trajectory.** It is meant to trim the frozen tail, but it also sorts the rows by robot 1's x position. The "final" state is then the row with the largest x₁, and `t` is simply cut to the same length.
4. **The flocking graph is hard-coded for 6 robots.** `Flock.m` sets `numRobots = 6` on its own, separately from `N_agents` in `Main.m`.
5. **Units don't match in the radial repulsion.** The spill center `Pc` is divided by 100 (unit square), but robot positions are in grid cells (0–100).
6. **Oil leaks at the grid edge.** Edge cells in `step()` lose mass without redistributing it, and `initialize()` does no bounds checks.
7. **`SimpleSpillEstimator` is unfinished.** Its `step`/`current_map` methods refer to an `env` property that doesn't exist, and it needs the Curve Fitting Toolbox.
8. **Performance.** Every iteration prints `ITERATION`, `x_v_a` and `forces_arp` (lines without semicolons), and the diffusion step loops over every cell. Vectorizing it (e.g. with `conv2`) and removing the prints would speed things up a lot.
9. **An empty extra figure appears.** `hold off` at the top of `Main.m` opens a blank figure before the real plots.
10. **The flocking potential's derivative is slightly off.** `dV/dr` is coded as `2(r·ln r − 1)/r²`. The exact derivative of ln²r + 1/r is `(2r·ln r − 1)/r²`.

## Reproducibility notes

The results above were produced headlessly with GNU Octave 8.4, using a copy of this folder with these Octave-only changes. None of them changes the algorithm:

* `h.TextPrims` (MATLAB-internal contour labels) is wrapped in `try/catch`.
* `h.LineWidth = 2` becomes `set(h,'LineWidth',2)`.
* `scatter(x,y,'+','k')` becomes `scatter(x,y,[],'k','+')`.
* A small `mink` shim is added.
* A random seed is set, and `Pot.m` saves its logged metrics to a `.mat` file for the summary table.

In MATLAB the original files should run as they are.
