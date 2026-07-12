# Quickstart — reproduce the M1 results

Copy-paste. Needs Docker on a **native x86-64 Linux** box (not ARM / not an
M-series Mac — see the note at the bottom of the README). Nothing else to install.

## 1. One-time setup (~10 min, mostly the image download)

```bash
# the upstream dev image (ROS 2 Jazzy + Nav2 + Gazebo + oomwoo_one), ~9.5 GB
docker pull makerspet/oomwoo:jazzy-dev

# my M1 packages
git clone https://github.com/jayadevrana/oomwoo-m1-ros2 ~/oomwoo-dev

# a long-lived container with the packages mounted at /root/oomwoo-dev
docker rm -f oom 2>/dev/null
docker run -d --name oom -v ~/oomwoo-dev:/root/oomwoo-dev makerspet/oomwoo:jazzy-dev sleep infinity

# build them (~20 s)
docker exec oom bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros_ws/install/setup.bash && cd /root/oomwoo-dev && colcon build --symlink-install --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support'

chmod +x ~/oomwoo-dev/deploy/*.sh
```

## 2. Kidnapped-robot test (~4 min)

```bash
docker exec oom bash /root/oomwoo-dev/deploy/run_reloc_regression.sh
```

Teleports the robot to 10 random spots and recovers each. Prints per-trial lines
and a summary; exits 0 on pass. Expect:

```
RELOC_SUMMARY passed=10/10 success_rate=1.00 target=0.90 ... suite_pass=True
```

## 3. Coverage test (~20 min)

```bash
docker exec oom bash /root/oomwoo-dev/deploy/run_coverage_regression.sh
```

Sweeps the room, then a gap-fill pass. Prints `COVERAGE_REPORT` lines and a
summary; exits 0 on pass. Expect:

```
COVERAGE_SUMMARY coverage=0.90.. efficiency=0.86.. ... pass=True
```

## That's it

Both are fully headless (`--headless-rendering`, software GL) — no display, no
GPU, drops straight into CI. The JSON reports land at `/root/reloc_report.json`
and `/root/coverage_report.json` inside the container.

To watch it live instead of waiting on the summary:

```bash
docker exec oom tail -f /tmp/reloc_regression.log      # or /tmp/coverage_regression.log
```

## Notes

- **Speed:** on 4 vCPU the sim runs ~real-time. On fewer cores it's slower but
  the metrics are unaffected (they're measured in sim time).
- **The two launch files** if you want to drive them yourself instead of the
  scripts: `ros2 launch oomwoo_sim_support relocalize_regression.launch.py` and
  `... coverage_regression.launch.py` (source the three setup.bash files first).
- **Clean up:** `docker rm -f oom` when done.
