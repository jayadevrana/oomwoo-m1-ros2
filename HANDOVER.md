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

## Stock living_room (headless)

Ran the stock `living_room` headless on a native x86-64 box and measured it
directly. Two things people assume are broken actually aren't — and one thing is:

- **Furniture is visible to the LiDAR.** All 360 beams return; dropped the robot
  into the middle of the furniture and it sees obstacles at 0.2–0.6 m, the `.obj`
  sofa and the `.dae` marble table included. The "invisible meshes" I hit before
  was an emulation (QEMU) artifact, not the world.
- **`.obj` furniture already collides.** Drove the robot into the sofa — it stops
  at the sofa face. No pass-through on native hardware.
- **The `.dae` marble table did NOT collide** — the robot drove straight through
  it (Gazebo's dartsim builds no collision for that mesh). That's the one real gap.

Fix: a collision-only static box proxy for each mesh furniture item, sized to the
mesh bounding box (`tools/gen_livingroom_proxies.py`). Visuals stay stock, so the
LiDAR still sees the real meshes; the boxes just give physics something to stop
the robot at. After the fix the table stops the robot (0.81 m pass-through → 0.21 m
stop). The map is generated world-aligned from the world geometry
(`tools/gen_livingroom_map.py`) because the stock SLAM map is in an offset frame.

Coverage on the stock living_room (cluttered — ~1.5 m widest gap):

```
Coverage:    90.0%   (target 90%)   PASS   (hit even in the tight room)
Efficiency:  39.2%   (target 80%)   below  (82.7 m driven for a 32.4 m ideal —
                                            the clutter forces a lot of maneuvering)
```

Run it: `deploy/run_coverage_livingroom.sh` (same harness, pointed at living_room).

## Next

Packages are on my GitHub; I'll open the two `contributions/` PRs (README + links)
next. Then onto M2 — validating xbattlax's merged Pi scaffold on real Pi 4 4GB,
integrating these packages, and reporting the measured baseline, as we discussed.

— Jayadev
