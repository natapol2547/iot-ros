#!/usr/bin/env bash
# Sourced by pixi on every `pixi run` / `pixi shell`. Puts this workspace's
# built packages on the ROS search path, if they've been built yet.
if [ -f "${PIXI_PROJECT_ROOT:-.}/install/setup.bash" ]; then
  source "${PIXI_PROJECT_ROOT:-.}/install/setup.bash"
fi
