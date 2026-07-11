#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Populate /overlay_ws/src either from the self-hosted git repo (default) or from
# the local build context copied to /tmp/local_src (USE_LOCAL=1). Kept in a
# script so the Dockerfile stays readable and the choice is one build-arg.
set -euo pipefail

mkdir -p /overlay_ws/src

if [ "${USE_LOCAL:-0}" = "1" ]; then
    echo "[_get_pkgs] using local build context"
    cp -r /tmp/local_src/oomwoo_coverage \
          /tmp/local_src/oomwoo_nav_localize \
          /tmp/local_src/oomwoo_sim_support /overlay_ws/src/
else
    echo "[_get_pkgs] cloning ${PKG_REPO} (${PKG_BRANCH})"
    git clone --depth 1 -b "${PKG_BRANCH}" "${PKG_REPO}" /overlay_ws/src/oomwoo_m1
fi

echo "[_get_pkgs] packages present:"
find /overlay_ws/src -maxdepth 2 -name package.xml -printf '  %h\n'
