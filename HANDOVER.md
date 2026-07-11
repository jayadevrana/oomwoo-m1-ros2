# OOMWOO — Milestone 1 Handover

Hi Ilia,

Here's where M1 stands and what's in this bundle. I built the simulation
implementation and the headless regression harness for both behaviours —
whole-map coverage cleaning and kidnapped-robot relocalization — against the
`oomwoo_one` robot and the `SOFTWARE_INTERFACES.md` contract.

Everything follows the structure you asked for: the ROS 2 packages are meant to
live in my own GitHub, the image builds from a fork of `oomwoo-install`, and the
`contributions/.../jayadevrana/` PRs are just READMEs that link back to the
packages.

## What's in here

```
oomwoo-m1-jayadevrana/
├── src/
│   ├── oomwoo_coverage/        # boustrophedon coverage planner (Nav2 NavigateThroughPoses)
│   ├── oomwoo_nav_localize/    # kidnap detection + AMCL global re-init + explore recovery
│   └── oomwoo_sim_support/     # headless bringup, ground-truth, meters, CLI regression runners
├── deploy/
│   ├── Dockerfile              # fork of oomwoo-install — layers these packages onto oomwoo:jazzy-dev
│   ├── _get_pkgs.sh
│   ├── run_coverage_regression.sh   # one-command headless coverage test (exit 0 = pass)
│   └── run_reloc_regression.sh      # one-command headless relocalization test (exit 0 = pass)
├── docs/
│   ├── PR_cleaning-jobs_README.md   # ready for contributions/cleaning-jobs/jayadevrana/
│   └── PR_nav-localize_README.md    # ready for contributions/nav-localize/jayadevrana/
├── README.md                  # full technical README
├── LICENSE                    # Apache-2.0
└── HANDOVER.md                # this file
```

## How it works, briefly

- **Coverage:** loads the saved `living_room` map, brings up Nav2 + AMCL
  headless, plans a back-and-forth sweep over the robot's *reachable* free space
  (flood-filled from its pose, so no waypoint ever lands inside a wall), and
  drives it through Nav2. Coverage and efficiency are scored from the robot's
  **ground-truth** pose, not from what the robot thinks — swept area over
  reachable area, and ideal-sweep-length over actual-path-length.
- **Relocalization:** starts localized, gets teleported ("kidnapped"), detects
  the loss, asks AMCL to scatter its particles globally, then spins and explores
  (driving with LiDAR obstacle-avoidance) until the pose re-converges. Scored
  against the known teleport target over 10 randomized trials.
- Both run fully headless from the CLI (`--headless-rendering`), so they drop
  straight into CI.

## Honest status

The **implementation and the regression harness are complete and reproducible.**
The one thing I'm finalising is the last measured run to lock in the headline
numbers (coverage %, efficiency %, relocalization success rate). I hit an
environment issue that's worth flagging so you can reproduce cleanly:

- The `oomwoo:jazzy-dev` image is amd64. On an ARM host under emulation the
  bridged `/clock` (1 kHz) arrives out of order and jumps backwards, which
  constantly clears Nav2's TF buffers and stops the planner from ever
  activating. **On a native x86-64 host this goes away completely** — I've
  confirmed the full stack activates, AMCL localizes, the coverage goal is
  accepted, and the kidnap→recover loop runs. I'm just completing the timed
  runs to fill the numbers into the two PR READMEs and the results table.

So: run it on native x86-64 (which is your CI target anyway) and the two
`deploy/run_*_regression.sh` scripts will print the pass/fail and write the JSON
reports.

## Next steps on my side

1. Finish the measured runs on x86-64 and fill the results tables.
2. Push the packages to my GitHub, build the forked image, and open the two
   `contributions/` PRs with the linking READMEs.

Happy to walk through any of it on a call. Thanks!

— Jayadev
