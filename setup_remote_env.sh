#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/piper_tele_ws/install/setup.bash"
export PYTHONPATH="$HOME/piper_tele_ws/src:$PYTHONPATH"
export LD_LIBRARY_PATH="$HOME/piper_tele_ws:$LD_LIBRARY_PATH"
