# Coverage Cleaning — jayadevrana

Contribution to [`cleaning-jobs`](../../): **regular / auto whole-map coverage
cleaning** for `oomwoo_one` in Gazebo, driven by Nav2, with a headless CLI
regression that verifies the acceptance metrics.

> Per project convention, the ROS 2 packages are **self-hosted**; this README
> links to them rather than vendoring the code here.

## Self-hosted packages

- **`oomwoo_coverage`** — boustrophedon coverage planner →
  `https://github.com/jayadevrana/oomwoo-m1-ros2` (`/oomwoo_coverage`)
- **`oomwoo_sim_support`** — headless bringup, ground-truth coverage meter,
  regression runner → same repo (`/oomwoo_sim_support`)
- Docker image (fork of `oomwoo-install`): build instructions in the repo's
  `deploy/Dockerfile`.

## What it does

- Loads the saved `living_room` map, brings up Nav2 + AMCL headless.
- Plans a **back-and-forth (boustrophedon) sweep** restricted to the robot's
  *reachable* free space (flood-filled from the robot pose, so no waypoint is
  ever stranded behind a wall), spaced by the cleaning swath with configurable
  overlap, and executes it via Nav2 `NavigateThroughPoses`.
- Measures coverage from the robot's **ground-truth** pose (not the planner's
  belief): swept-area / reachable-area, plus a path-efficiency ratio.

## Acceptance metrics

| Metric | Target | How measured |
|---|---|---|
| Coverage | ≥ 90 % | reachable free cells swept by the cleaning disk along the true path |
| Efficiency | ≥ 80 % | ideal gap-free sweep length / actual path length |

## Run (headless, CLI)

```bash
# inside the built image / overlay workspace
./deploy/run_coverage_regression.sh        # exit 0 == PASS; writes coverage_report.json
# or manually:
ros2 launch oomwoo_sim_support coverage_regression.launch.py
ros2 run  oomwoo_sim_support coverage_regression_runner
```

## Test results

_x86-64 Linux, headless. Populated from the latest run._

```
coverage=____  efficiency=____  pass=____
```

## Notes / scope

- M1 scope: whole-map regular coverage + its headless regression. Spot mode,
  room segmentation, no-go zones, and job pause/resume are follow-on work in the
  `cleaning-jobs` RFC.
- Interfaces follow
  [SOFTWARE_INTERFACES.md](../../../docs/SOFTWARE_INTERFACES.md).
- Requires a **native x86-64** host (ARM emulation has an unstable sim clock).

— Jayadev Rana · Apache-2.0
