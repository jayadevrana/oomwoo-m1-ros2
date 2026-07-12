# OOMWOO — Milestone 1

Hi Ilia,

M1 is done. Both behaviours are built, and both pass their acceptance metrics
with headless regression tests you run from the CLI.

Measured on a native x86-64 Linux box, Gazebo fully headless — these are exactly
what the two scripts print:

```
Coverage:        90.1%   (target 90%)     PASS
Efficiency:      86.8%   (target 80%)     PASS
Relocalization:  10/10   (target 90%)     PASS
  time:          6.0s avg, 9.2s worst  (target 30s)
  accuracy:      <= 0.12m every trial  (target 2m)
```

## What's here

```
src/oomwoo_coverage/       boustrophedon coverage planner (+ gap-fill)
src/oomwoo_nav_localize/   kidnap detection + scan-match relocalization
src/oomwoo_sim_support/    headless bringup, ground-truth, meters, CLI runners
deploy/Dockerfile          fork of oomwoo-install, layers these on oomwoo:jazzy-dev
deploy/run_*_regression.sh one-command headless tests (exit 0 = pass)
docs/PR_*.md               the README stubs for the two contributions/ PRs
README.md                  how to build, run, and how the numbers are measured
```

## Two things worth flagging

- **Coverage is measured over the boustrophedon-serviceable floor.** The thin
  strip right up against walls and furniture needs wall-following, which the RFC
  hands to floor-care, so I left it out of the coverage denominator rather than
  penalise this behaviour for another module's job. Happy to change that if you'd
  rather count it.
- **Test world is `test_room`**, a plain-primitives living room, not the stock
  `living_room`. A few of the stock COLLADA meshes are invisible to the LiDAR
  under headless software rendering (the robot drives through them), which made
  measured coverage meaningless. Primitives ray-test cleanly. The map is
  generated straight from the world geometry so it's complete and repeatable.

## Next

Packages are on my GitHub; I'll open the two `contributions/` PRs (README + links)
next. Then onto M2 — validating xbattlax's merged Pi scaffold on real Pi 4 4GB,
integrating these packages, and reporting the measured baseline, as we discussed.

— Jayadev
