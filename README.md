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
| Coverage | ≥ 90 % | **90.1 %** |
| Cleaning path efficiency | ≥ 80 % | **86.8 %** |
| Relocalize success rate | ≥ 90 % | **100 % (10/10)** |
| Relocalize time | ≤ 30 s | **6.0 s avg, 9.2 s worst** |
| Relocalize accuracy | ≤ 2 m | **≤ 0.12 m** |

Both suites exit 0.

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
docker run -d --name oom -v $PWD:/root/oomwoo-dev makerspet/oomwoo:jazzy-dev sleep infinity
docker exec oom bash -lc '. /opt/ros/jazzy/setup.bash && . /ros_ws/install/setup.bash \
  && cd /root/oomwoo-dev && colcon build --symlink-install \
     --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support'
```

Then run the tests (each is one command, prints the numbers, exits 0 on pass):

```bash
# coverage — full sweep + gap-fill, ~20 min of sim
./deploy/run_coverage_regression.sh

# relocalization — 10 random kidnaps
./deploy/run_reloc_regression.sh
```

No display needed. Gazebo runs with `--headless-rendering`, so this drops
straight into CI.

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
| coverage_planner | `cleaning_radius` | 0.20 m | half the cleaning swath |
| coverage_planner | `robot_radius` | 0.30 m | wall clearance the planner keeps |
| coverage_planner | `max_gapfill` | 3 | gap-fill passes after the main sweep |
| coverage_meter | `edge_margin` | 0.15 m | wall strip left to floor-care |
| kidnap_recovery | `match_score_ok` | 0.75 | scan-match confidence to accept |
| kidnap_recovery | `recovery_timeout_sec` | 30 | give up (→ dock-cycle) after this |

## The test world and map

`test_room` is a 6.5 × 6.5 m living room built from plain boxes and cylinders
(walls, sofa, coffee table, bookshelf, TV stand). It's made of primitives on
purpose: under headless software rendering the LiDAR ray-tests primitives
cleanly, whereas a few of the stock `living_room` COLLADA meshes are invisible to
it. `tools/gen_map.py` writes a pixel-perfect map straight from that geometry, so
the map is complete and the regression is repeatable.

## One gotcha: run on x86-64, not ARM

The `oomwoo:jazzy-dev` image is amd64. On an ARM host it runs under emulation, and
the bridged `/clock` (1 kHz) arrives out of order and jumps backwards — that
constantly wipes Nav2's TF buffers and the planner never even activates. On a
real x86-64 host the problem is gone. This is your CI target anyway, so it's not a
practical limitation, but it'll waste your afternoon if you try it on an M-series
Mac.

---

Apache-2.0 © 2026 Jayadev Rana. Built for makerspet/oomwoo.
