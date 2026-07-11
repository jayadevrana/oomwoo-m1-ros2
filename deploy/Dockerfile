# OOMWOO M1 — coverage + relocalization overlay image (Jayadev Rana)
# SPDX-License-Identifier: Apache-2.0
#
# This is the Dockerfile for a *fork* of makerspet/oomwoo-install. It layers the
# self-hosted M1 packages on top of the upstream makerspet/oomwoo:jazzy-dev
# development image (which already provides ROS 2 Jazzy, Nav2, Gazebo, AMCL,
# slam_toolbox, the kaiaai platform and the oomwoo_one robot description).
#
# Build:
#   docker build -t jayadevrana/oomwoo-m1:jazzy \
#     --build-arg PKG_REPO=https://github.com/jayadevrana/<packages-repo>.git \
#     -f deploy/Dockerfile .
#
# Run (headless regression, no GUI needed):
#   docker run --rm jayadevrana/oomwoo-m1:jazzy \
#     ros2 launch oomwoo_sim_support coverage_regression.launch.py
#
# Two ways to get the packages in, selected by USE_LOCAL:
#   * USE_LOCAL=0 (default): git clone PKG_REPO   (the self-hosted repo)
#   * USE_LOCAL=1          : COPY ./src           (local build context)

ARG BASE_IMAGE=makerspet/oomwoo:jazzy-dev
FROM ${BASE_IMAGE}

ARG PKG_REPO=https://github.com/jayadevrana/oomwoo-m1-ros2.git
ARG PKG_BRANCH=main
ARG USE_LOCAL=0

# Overlay workspace, kept separate from the upstream /ros_ws so the base image
# stays intact and this layer is easy to rebuild.
WORKDIR /overlay_ws
COPY deploy/_get_pkgs.sh /tmp/_get_pkgs.sh
COPY src /tmp/local_src
RUN chmod +x /tmp/_get_pkgs.sh && /tmp/_get_pkgs.sh

RUN . /opt/ros/jazzy/setup.sh \
 && . /ros_ws/install/setup.sh \
 && rosdep install --from-paths src --ignore-src -y || true \
 && colcon build --symlink-install \
      --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support \
 && rm -rf build log /tmp/local_src

# Source the overlay for interactive shells and launches.
RUN echo ". /overlay_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /overlay_ws
