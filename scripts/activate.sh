#!/usr/bin/env bash
# Sourced by pixi on every `pixi run` / `pixi shell`. Puts this workspace's
# built packages on the ROS search path, if they've been built yet.
#
# install/setup.bash chains to the pixi environment it was built with, so it is
# only sourced when that is the active environment. Sourcing a build of the
# default environment under `-e robot` (or the reverse) would put the other
# environment's compilers and libraries first on PATH.
iot_setup="${PIXI_PROJECT_ROOT:-.}/install/setup.bash"
if [ -f "$iot_setup" ]; then
  if grep -q "^COLCON_CURRENT_PREFIX=\"${CONDA_PREFIX}\"$" "$iot_setup"; then
    source "$iot_setup"
  else
    iot_env="${PIXI_ENVIRONMENT_NAME:+ -e $PIXI_ENVIRONMENT_NAME}"
    echo "activate.sh: install/ was built with another pixi environment and is" \
         "not sourced. Rebuild with: pixi run$iot_env clean && pixi run$iot_env build" >&2
    unset iot_env
  fi
fi
unset iot_setup
