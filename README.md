# OOMWOO M1 — Coverage Cleaning & Kidnapped-Robot Relocalization (ROS 2 Jazzy)

Self-hosted ROS 2 packages implementing Milestone 1 of the
[makerspet/oomwoo](https://github.com/makerspet/oomwoo) open-source robot vacuum:

1. **Regular / auto cleaning** — plan a coverage path over a saved map and clean
   the whole reachable floor with Nav2. Target: **≥ 90 % coverage, ≥ 80 %
   efficiency**.
2. **Kidnapped-robot relocalization** — detect when the robot is lost / picked
   up and moved, then recover its pose on the saved map. Target: **relocalize
   within 30 s, to within 2 m, ≥ 90 % of the time**.
3. **Headless regression tests** for both, runnable from the CLI with Gazebo in
   headless mode (CI-friendly).

Built against the OOMWOO
[SOFTWARE_INTERFACES.md](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)
contract and the `oomwoo_one` Gazebo robot. Apache-2.0.

---

## Packages

| Package | Role |
|---|---|
| `oomwoo_coverage` | Boustrophedon coverage **planner** — plans a back-and-forth sweep over the robot's reachable free space and executes it through Nav2 `NavigateThroughPoses`. Owns the "what/where to clean" decision. |
| `oomwoo_nav_localize` | **Kidnap recovery** — detects localization loss (AMCL covariance / external pickup signal), triggers AMCL global re-initialization, and actively explores (spin + drive with obstacle avoidance) until the pose re-converges; reports a clear success/failure status. |
| `oomwoo_sim_support` | **Headless bringup, ground-truth measurement, and regression harnesses.** Sim + Nav2 + AMCL launch, a ground-truth pose publisher (from noise-free sim odometry), an honest coverage meter, a kidnap injector (Gazebo teleport), and the CLI regression runners. |

### Node map

```
oomwoo_coverage/coverage_planner   /map, /amcl_pose, /coverage/ratio → NavigateThroughPoses
oomwoo_nav_localize/kidnap_recovery /amcl_pose, /scan, /kidnap_trigger → /cmd_vel, /reinitialize_global_localization
oomwoo_sim_support/ground_truth     /odom → /ground_truth/pose      (map-frame truth)
oomwoo_sim_support/coverage_meter   /map, /ground_truth/pose → coverage %, efficiency (COVERAGE_REPORT)
oomwoo_sim_support/kidnap_injector  ~/kidnap service → gz set_pose teleport + /kidnap_trigger + ~/target_pose
oomwoo_sim_support/reloc_regression_runner   drives N kidnap trials, scores vs truth, writes JSON report
```

---

## Interfaces (per SOFTWARE_INTERFACES.md)

**Consumed:** `/scan`, `/odom`, `/tf`, `/map`, `/amcl_pose`, Nav2
`navigate_through_poses`, AMCL `reinitialize_global_localization`.
**Produced:** `/cmd_vel` (arbitrated — see below), `/coverage_meter/ratio`,
`/coverage_meter/efficiency`, `~/localization_status`, `/ground_truth/pose`.

**`/cmd_vel` arbitration.** During coverage, Nav2's controller is the only
velocity source. During relocalization, the sim runs **without** the Nav2 nav
servers and the recovery node is the only velocity source; it publishes
`~/recovering` so an integrator can gate any other motion source.

Frames follow REP-103: `map → odom → base_footprint → base_link → base_scan`.

---

## How coverage & efficiency are measured (honestly)

Metrics come from the robot's **ground-truth pose**, never from the planner's own
belief:

- **Coverage** = (reachable free cells swept by a `cleaning_radius` disk along the
  true path) / (reachable free cells). "Reachable" is a flood fill from the
  robot's start cell, so sealed-off voids never inflate the score.
- **Efficiency** = `ideal_path_length / actual_path_length`, where
  `ideal_path_length = reachable_area / swath_width` (a perfect gap-free
  boustrophedon). At constant speed this equals time efficiency.

Ground truth is the sim's noise-free odometry (`/odom`), whose frame is pinned to
the robot's spawn = the SLAM map origin, so `/odom` xy == map-frame truth.

Relocalization is scored against the **known teleport target** the kidnap injector
commands (odometry does not jump on teleport, so the injector's target is the
authoritative post-kidnap truth).

---

## Prerequisites

- Docker, on an **x86-64 Linux** host. (See *Environment notes* — ARM/emulated
  hosts have an unstable sim clock and are not supported for the closed-loop run.)
- The upstream dev image `makerspet/oomwoo:jazzy-dev` (pulled automatically by the
  Dockerfile below).

## Build the image (fork of oomwoo-install)

```bash
# from a fork of makerspet/oomwoo-install, with this repo's deploy/ + src/ present
docker build -t jayadevrana/oomwoo-m1:jazzy \
  --build-arg USE_LOCAL=1 \
  -f deploy/Dockerfile .
# or pull the packages from the self-hosted repo instead of the local context:
#   --build-arg USE_LOCAL=0 --build-arg PKG_REPO=https://github.com/jayadevrana/oomwoo-m1-ros2.git
```

Or build the packages inside the stock image (overlay workspace):

```bash
docker run -d --name oom -v $PWD:/root/oomwoo-dev makerspet/oomwoo:jazzy-dev sleep infinity
docker exec oom bash -c '. /opt/ros/jazzy/setup.bash && . /ros_ws/install/setup.bash \
  && cd /root/oomwoo-dev && colcon build --symlink-install \
     --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support'
```

---

## Run the regression tests (headless, CLI)

Everything runs with `--headless-rendering` (offscreen software GL) — no display,
no GUI. Suitable for CI.

**Coverage:**
```bash
ros2 launch oomwoo_sim_support coverage_regression.launch.py
# watch COVERAGE_REPORT lines: coverage=… efficiency=… reachable_cells=… sim_t=…
```

**Relocalization (10 kidnap trials, writes a JSON report):**
```bash
ros2 launch oomwoo_sim_support relocalize_regression.launch.py &
ros2 run oomwoo_sim_support reloc_regression_runner   # exits 0 iff suite passes
cat /root/reloc_report.json
```

The two `deploy/run_*_regression.sh` scripts are the CI entry points — each is a
single command with a pass/fail exit code and a JSON report. (pytest /
launch_testing wrappers around the same runners are planned as a follow-up.)

---

## Configuration (key parameters)

| Node | Param | Default | Meaning |
|---|---|---|---|
| coverage_planner | `cleaning_radius` | 0.16 m | half the effective clean swath |
| coverage_planner | `row_overlap` | 0.10 | fraction of swath overlapped between rows |
| coverage_planner | `coverage_target` | 0.90 | stop when reached |
| coverage_meter | `cleaning_radius` | 0.16 m | must match the planner |
| kidnap_recovery | `lost_cov_trace` | 0.6 | AMCL covariance-trace threshold to declare "lost" |
| kidnap_recovery | `ok_cov_trace` | 0.25 | trace below which the pose is "re-converged" |
| kidnap_recovery | `recovery_timeout_sec` | 30 | fail (→ dock-cycle fallback) after this |
| kidnap_recovery | `drive_speed` / `spin_speed` | 0.16 / 0.9 | explore motion during recovery |
| kidnap_injector | `min_jump` | 1.5 m | minimum teleport distance |

---

## Test results

_Populated from the latest headless run on x86-64 Linux._

| Metric | Target | Result |
|---|---|---|
| Coverage | ≥ 90 % | _TBD_ |
| Efficiency | ≥ 80 % | _TBD_ |
| Relocalization success rate | ≥ 90 % | _TBD_ |
| Relocalization time / accuracy | ≤ 30 s / ≤ 2 m | _TBD_ |

---

## Environment notes / troubleshooting

- **Run on native x86-64 Linux.** On ARM hosts the amd64 image runs under
  emulation, where the bridged `/clock` (1 kHz) is delivered out-of-order under
  load and jumps backward thousands of times per run, continuously clearing every
  node's TF buffer — Nav2's costmaps never stabilize and `planner_server` won't
  activate. The provided `living_room_fast.world` coarsens physics to 200 Hz to
  reduce this, but a native x86 host is required for the closed-loop tests.
- **RAM.** The full Nav2 stack + Gazebo peaks ~4–5 GB. The relocalization stack
  is much lighter (no nav servers, ~1.1 GB). On constrained hosts, cap the
  container CPU (`docker update --cpus 1.5 oom`) so ROS can't starve the host.
- **`RTPS_TRANSPORT_SHM … open_and_lock_file failed`** warnings are harmless —
  FastDDS shared memory is disabled in the base image; DDS falls back to UDP.
- **`map_server: bad file map.yaml`** — the map path must be absolute; this repo's
  bringup passes the packaged `maps/living_room.yaml` directly.

---

## License

Apache-2.0 © 2026 Jayadev Rana. Developed for the makerspet/oomwoo project.
