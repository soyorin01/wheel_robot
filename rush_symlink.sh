#!/usr/bin/env bash
# 把 install 里的 rush.py 恢复成指向 src 的软链接（只动这一个文件）。
# 什么时候需要：跑过 colcon build 之后，软链接会被覆盖成普通文件拷贝。
set -e
SRC=/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py
DST=/home/orangepi/wheel_robot/install/wheel_robot/lib/wheel_robot/rush.py
chmod 755 "$SRC"
ln -sfn "$SRC" "$DST"
ls -l "$DST"
echo "OK：以后改 $SRC 立即生效（不用 build）"
