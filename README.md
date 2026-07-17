# OOMWOO — Coverage Cleaning & Kidnapped-Robot Recovery (ROS 2 Jazzy)

Milestone 1 for the [makerspet/oomwoo](https://github.com/makerspet/oomwoo)
open-source robot vacuum. Two behaviours, both with headless regression tests you
run from the command line:

1. **Auto cleaning** — plan a back-and-forth path over a saved map and clean the
   whole floor with Nav2.
2. **Kidnapped-robot recovery** — the robot gets picked up and dropped somewhere
   else; it figures out where it is again on the saved map.

Everything runs on the `oomwoo_one` robot in Gazebo, against the
[SOFTWARE_INTERFACES.md](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)
contract. Apache-2.0.

## Results

Measured on native x86-64 Linux (4 vCPU), Gazebo fully headless, exactly what the
two scripts print. Numbers are from the robot's true pose, not what the robot
thinks — see "How it's measured" below.

| Behaviour | Target | Measured |
|---|---|---|
| Coverage (uncapped, sweep run to completion) | ≥ 90 % | **94.5 %** |
| Efficiency at the 90 % crossing | ≥ 80 % | **84.8 %** (785 s) |
| Efficiency incl. finishing the last ~4.5 % | — | 71.3 % (reported, not gated) |
| Relocalize success rate | ≥ 90 % | **100 % (10/10)** |
| Relocalize time | ≤ 30 s | **6.0 s avg, 9.2 s worst** |
| Relocalize accuracy | ≤ 2 m | **≤ 0.12 m** |

Both suites exit 0. Coverage is **not target-capped**: the planner runs the
full sweep + gap-fill to completion and the report shows where it genuinely
ends (`end_reason=sweep_complete`), so 94.5 % is a measured ceiling, not a
stop condition. The two gates are one contract condition — reach ≥90 % at
≥80 % efficiency — so efficiency is judged at the moment coverage first
crosses 90 %; pushing on to 94.5 % costs extra spot-revisit path (diminishing
returns), which the report discloses as `efficiency_final`. The coverage
denominator is built from the **true body radius (0.1745 m)**, never the
planner's clearance — floor a timid planner won't enter counts against it.

## The packages

| Package | What it does |
|---|---|
| `oomwoo_coverage` | The coverage planner. Lays out a boustrophedon (lawnmower) sweep over the reachable floor, drives it waypoint-by-waypoint through Nav2, then does a short gap-fill pass over anything the sweep missed. |
| `oomwoo_nav_localize` | The recovery behaviour. Detects it's lost, does a one-shot scan-match against the map to find itself, seeds AMCL there, and confirms. |
| `oomwoo_sim_support` | Everything to test the above headless: the sim bringup, a ground-truth publisher, the coverage meter, the kidnap injector, and the two CLI regression runners. |

## Run it

You need Docker on an **x86-64 Linux** box (see the note at the bottom about ARM).
Build the image once — it layers these packages on top of the upstream
`makerspet/oomwoo:jazzy-dev` dev image:

```bash
docker build -t jayadevrana/oomwoo-m1:jazzy --build-arg USE_LOCAL=1 -f deploy/Dockerfile .
```

Or just build the packages inside the stock image:

```bash
docker run -it --name oom makerspet/oomwoo:jazzy-dev
# at the container's bash prompt — packages go in /ros_ws/src, the stock
# workspace, per oomwoo-install convention:
git clone https://github.com/jayadevrana/oomwoo-m1-ros2 /ros_ws/src/oomwoo-m1
cd /ros_ws && colcon build --symlink-install \
  --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support
source /ros_ws/install/setup.bash
```

Then run the tests (each is one command, prints the numbers, exits 0 on pass):

```bash
# coverage — full sweep + gap-fill, ~20 min of sim
bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_regression.sh

# relocalization — 10 random kidnaps
bash /ros_ws/src/oomwoo-m1/deploy/run_reloc_regression.sh

# coverage on the stock living_room
bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_livingroom.sh
```

No display needed — Gazebo runs with `--headless-rendering`, so this drops
straight into CI. To watch the identical sim with the Gazebo GUI, add
`gui:=true` to any launch **or to any of the three run scripts**
(`bash deploy/run_coverage_regression.sh gui:=true` — the scripts forward
extra args to the launch, and skip forcing software GL when the GUI is up).
To run against another vacuum model, use the
kaiaai convention (`kaia config robot.model <pkg>`) or pass
`robot_model:=<pkg>`; the regressions pin `oomwoo_one` for reproducibility.
`RUNS=3` before any script repeats it and prints the variance. If the meter
detects ground-truth teleports (unstable sim — e.g. Docker-on-Windows/WSL2),
the run exits 2 with "sim unstable" instead of reporting garbage numbers.

## How it's measured (so you can trust the numbers)

The catch with grading a localization robot is that you can't ask the robot how
well it did — it'll tell you it's doing great while sitting in the wrong spot. So
nothing here trusts the robot's own estimate:

- A **ground-truth node** republishes the simulator's true pose (from noise-free
  odometry, and it stays correct through teleports). The coverage meter and the
  kidnap scorer both compare against that, never against AMCL.
- **Coverage** = floor the cleaning disk actually passed over ÷ floor the robot
  can service. "Can service" is the reachable area minus the thin strip right
  against walls and furniture — that edge strip needs wall-following, which the
  OOMWOO RFC hands to the separate floor-care module, not to coverage.
- **Efficiency** = the length of a perfect gap-free sweep ÷ the distance actually
  driven. At constant speed that's the same as time efficiency.
- **Relocalization** is scored against the exact spot the injector teleported the
  robot to. Odometry doesn't jump on a teleport, so that target is the real
  ground truth.

## How recovery actually works

A plain particle filter (AMCL) struggles to wake up in a room with symmetry — it
locks onto a mirror-image pose and won't let go. So on a kidnap this node does a
**one-shot global scan-match**: it raycasts the map once into an expected-range
table, then correlates the live 360° scan against every free cell and heading.
The best match seeds AMCL, and a short in-place spin confirms the covariance
collapsed. That's why recovery is fast (a few seconds) and accurate (~0.1 m)
instead of drifting near the 30 s limit.

## Config worth knowing

| Node | Param | Default | Meaning |
|---|---|---|---|
| coverage_planner | `cleaning_radius` | 0.20 m | half the cleaning swath (see assumption below) |
| coverage_planner | `robot_radius` | 0.30 m | wall clearance the planner keeps (planning only — never used by the meter) |
| coverage_meter | `robot_radius` | 0.1745 m | the TRUE body radius; the meter's denominator is built from real geometry, not the planner's clearance, so floor a timid planner skips counts against the score |
| coverage_planner | `max_gapfill` | 3 | gap-fill passes after the main sweep |
| coverage_meter | `edge_margin` | 0.15 m | wall strip left to floor-care |
| kidnap_recovery | `match_score_ok` | 0.75 | scan-match confidence to accept |
| kidnap_recovery | `recovery_timeout_sec` | 30 | give up (→ dock-cycle) after this |

**Explicit assumption — the cleaning swath.** `cleaning_radius = 0.20` means a
0.40 m swath on a 0.349 m-wide robot: wider than the body. That is only true if
side-brush reach counts as "cleaned"; the planner spaces rows 0.38 m apart on
that assumption. If the real brush covers less, set `cleaning_radius` to the
actual half-swath — rows tighten, the run gets longer, and the score reflects
the real machine. The sim can't validate the brush; this parameter is where
that physical fact enters.

## The test world and map

`test_room` is a 6.5 × 6.5 m living room built from plain boxes and cylinders
(walls, sofa, coffee table, bookshelf, TV stand). It's the primary regression
world because it's fully deterministic — `tools/gen_map.py` writes a pixel-perfect
map straight from that geometry, so the map is complete and the run is repeatable.

## It also runs on the stock living_room

```bash
bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_livingroom.sh
```

The stock `living_room` world and models are used **exactly as upstream ships
them** — furniture visuals, mesh collisions, everything. dartsim builds the
`.dae`/`.obj` mesh collisions correctly headless (identically to running with
the GUI: the Gazebo server does the physics either way), so the robot drives
under the marble table, threads between its legs, and stops at real geometry.
Earlier revisions of this repo shipped collision workarounds (box proxies,
then primitive leg overrides) built on a faulty test harness — both were wrong
and both are gone.

The one thing generated here is the **map**: the stock SLAM map is in a frame
offset from the gz world, so `tools/gen_livingroom_map.py` produces a
world-aligned map by slicing the stock collision meshes at the robot's height
band (2–20 cm). Open-under furniture contributes only its legs, so the floor
beneath the table counts as cleanable — which is the whole point of a vacuum.

Measured on the **pure stock world** (no overrides), sweep run to completion
with the true-geometry meter: **89.3 % coverage** (`end_reason=sweep_complete`
— the planner genuinely exhausted its sweep + gap-fill passes), efficiency
32.0 %, sim stable (`pose_jumps=0`). Consistent with earlier runs (88.9 %,
override-era 89.7 %) — the stock mesh collisions were doing the job all along.
The room is tight (max obstacle clearance ~0.78 m anywhere, by distance
transform of the map), so efficiency lands well below the open
`test_room`'s by design; the last ~11 % is pockets Nav2's local costmap can't
enter. The script prints this and writes `coverage_report.json`.

The decoupling's measured effect (reproduce with
`tools/compute_denominator.py`): switching the meter from the planner's
clearance to the true body radius grows the denominator 5647 → 5661 cells
(+0.25%) here and 13637 → 13696 (+0.43%) in test_room — i.e. the coupled
meter *was* slightly inflating scores, in the direction the review predicted;
the magnitude just turns out to be fractions of a percent. Fixed regardless:
the meter is pinned to 0.1745 and no planner setting can shrink the
denominator again.

## One gotcha: run on x86-64, not ARM

The `oomwoo:jazzy-dev` image is amd64. On an ARM host it runs under emulation, and
the bridged `/clock` (1 kHz) arrives out of order and jumps backwards — that
constantly wipes Nav2's TF buffers and the planner never even activates. On a
real x86-64 host the problem is gone. This is your CI target anyway, so it's not a
practical limitation, but it'll waste your afternoon if you try it on an M-series
Mac.

---

Apache-2.0 © 2026 Jayadev Rana. Built for makerspet/oomwoo.
