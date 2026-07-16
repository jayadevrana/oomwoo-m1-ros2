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

## Stock living_room (headless) — stock world, untouched

You were right on both counts, and the repo now reflects it:

- **The stock world and models are used exactly as you ship them.** dartsim
  builds the mesh collisions correctly headless — identical to the UI run,
  since the Gazebo server does the physics either way. My earlier claims
  ("no raw-mesh collision headless", and the box proxies / primitive-leg
  override built on them) came from a faulty test harness: its verdict was a
  distance threshold calibrated on the solid sofa, so a robot correctly
  driving *under* the open table got labeled "ghosting". Your Claude's
  diagnosis of that bug is exactly right. All overrides are removed.
- **The map** is the one generated artifact: the stock SLAM map is in a frame
  offset from the gz world, so `tools/gen_livingroom_map.py` builds a
  world-aligned map by slicing the stock collision meshes at the robot's
  height band (2–20 cm) — under-table floor counts as cleanable.
- **Wedge escape** (kept — it's behaviour, not a world change): if Nav2 gives
  up on several waypoints in a row inside inflated-lethal pockets, the planner
  reverses straight out open-loop and resumes the sweep.

Run it: `deploy/run_coverage_livingroom.sh`. Measured on the **pure stock
world** (no overrides): **88.9% coverage**, efficiency 32.9%, stable
(pose_jumps=0), ended on plateau. That's within a whisker of the earlier
override-era 89.7% — which confirms the override never did anything useful; the
stock mesh collisions were already correct. The tight room caps efficiency, and
the last ~11% is pockets Nav2's local costmap can't enter.

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
- **Marble table + proxies** — all collision workarounds removed; the stock
  world is used untouched (details above). No charge for any of that rework.
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
