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
- **`test_room` stays the primary regression world** because it's fully
  deterministic: primitive geometry, and a map generated straight from that
  geometry, so the gate is complete and repeatable. (My original reason —
  "stock meshes are invisible to the LiDAR headless" — turned out to be an
  ARM-emulation artifact; the stock living_room renders fine on native x86-64
  and now has its own regression script, next section.)

## Stock living_room (headless) — no proxies, cleans under furniture

The box proxies from the first pass are gone — you were right that they blocked
exactly the floor a vacuum exists to clean. What replaced them, all verified
from the simulator's true pose:

- **Furniture is visible to the LiDAR headless.** All 360 beams return; the
  robot sees the sofa and the marble table at 0.2–0.6 m. The "invisible meshes"
  I reported earlier was an ARM-emulation artifact, and my "the .obj furniture
  already collides" claim was wrong too — a flaky test harness fooled me.
  The truth: Gazebo's dartsim engine skips *every* raw mesh collision headless
  ("Mesh construction ... not implemented"), .obj and .dae alike. Navigation
  still avoids furniture fine because the costmaps are LiDAR-driven.
- **The marble table now has real physics.** Converting the mesh doesn't work
  (dartsim skips it) and V-HACD convex decomposition bridges the open space
  under the top — measured: the robot wedged mid-table on a phantom hull. So
  the override model keeps the stock visual and carries primitive collisions
  measured from the mesh's own geometry: four floor-standing legs plus the
  tabletop slab. Drive-tested: the robot passes under the top, threads between
  the legs, and pins at a leg face within 1 cm of the predicted coordinate.
  Matches what you saw with the GUI on.
- **The map counts under-furniture floor as cleanable.** The stock SLAM map is
  in a frame offset from the gz world, so `tools/gen_livingroom_map.py`
  generates a world-aligned map by slicing every collision shape at the robot's
  height band (2–20 cm): open-under furniture contributes only its legs. The
  cleanable denominator grew ~9% the moment the proxies came out.
- **Wedge escape.** Entering tight pockets cuts both ways: Nav2's recoveries
  refuse to move inside inflated-lethal space, so a robot that squeezes in can
  strand there. The planner now detects back-to-back unreachable waypoints and
  reverses straight out open-loop, then resumes the sweep.

Run it: `deploy/run_coverage_livingroom.sh`. Measured on the stock room:

```
Coverage:    89.7%  of the full robot-height floor, under-furniture INCLUDED
Efficiency:  31.9%  (tight room — constant maneuvering; honest number)
Stability:   pose_jumps=0, no stuck events, ended on plateau
```

One caveat so the numbers read right: my earlier "90.0%" was scored against a
denominator that *excluded* everything under furniture (the proxies made it
unreachable). This run's denominator includes that floor — and the robot
actually cleans ~8.5% more real floor area than before, under the table
included. The last ~10% is a few pockets Nav2's local costmap genuinely can't
enter; that's the true limit of this robot in this room, not a measurement gap.

## Feedback round (post-M1 review)

Everything from your review, in:

- **Headless ↔ GUI switch** — `gui:=true` on any launch runs the identical sim
  with the Gazebo GUI (headless stays the default; software GL is only forced
  headless).
- **oomwoo-install conventions** — packages clone into `/ros_ws/src/oomwoo-m1`
  and build in the stock `/ros_ws` workspace; the Dockerfile and all docs
  follow the pull → run → bash-prompt → CLI flow of the existing tutorials.
- **`kaia config robot.model`** — launches resolve the robot description the
  kaiaai way (launch arg → `~/.kaiaai.yaml` → default). The regression scripts
  pin `oomwoo_one` so the gate is reproducible on any machine.
- **Marble table + proxies** — proxies removed, table collision authored from
  its own mesh geometry (details above). No charge for the proxy removal.
- **Measurement hardening** (from your Claude's findings) — the meter rejects
  implausible ground-truth jumps from the path length, counts them, and latches
  `sim_unstable`; the runner then aborts with a distinct exit code and a clear
  "re-run on native Linux" message instead of reporting garbage efficiency.
  `RUNS=N` repeats any suite and prints min/max/mean/stdev; the reloc report
  now includes time stddev/min/max per suite.

## Next

Packages are on my GitHub; I'll open the two `contributions/` PRs (README + links)
next. Then onto M2 — validating xbattlax's merged Pi scaffold on real Pi 4 4GB,
integrating these packages, and reporting the measured baseline, as we discussed.

— Jayadev
