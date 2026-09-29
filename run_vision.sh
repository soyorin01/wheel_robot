#!/bin/bash
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"

cd "$HOME/wheel_robot/src/wheel_robot/scripts"
exec python3 camera_viewer.py
