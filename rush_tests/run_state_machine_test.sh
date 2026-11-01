#!/bin/bash
# 无赛道状态机自检：用假 IMU/odom 把 ALIGN→RUSH→桥→WAIT_TURN→TURN→SIDEWALK
# →T_ALIGN→T_CROUCH→T_RUSH→POST_TUNNEL 整条推一遍。
# 安全：enable_motion:=false，不发任何运动指令；也要求底盘节点没在跑。
set -e
source /opt/ros/humble/setup.bash
source ~/wheel_robot/install/setup.bash
cd ~/wheel_robot
if [ -n "$(timeout 6 ros2 node list 2>/dev/null | tr -d " ")" ]; then
  echo "有 ROS 节点在跑，先关掉再来"; exit 9
fi
# 只改触发方式与时长，代码路径和正式版完全一致；不覆盖已部署文件
sed -e "s/^RUSH_TURN_TRIGGER = \"visual\"/RUSH_TURN_TRIGGER = \"odom\"/" \
    -e "s/^RUSH_TURN_START_DISTANCE = 1.20/RUSH_TURN_START_DISTANCE = 0.50/" \
    -e "s/^SIDEWALK_STANDSTILL_SECONDS = 5.0/SIDEWALK_STANDSTILL_SECONDS = 1.5/" \
    -e "s/^RUSH_TUNNEL_SECONDS = 2.60/RUSH_TUNNEL_SECONDS = 1.50/" \
    src/wheel_robot/scripts/rush.py > /tmp/rush_test.py
python3 "$(dirname "$0")/fake_sensors.py" > /tmp/fake.log 2>&1 &
FAKE=$!
sleep 1
timeout 20 python3 /tmp/rush_test.py --ros-args \
  --params-file src/wheel_robot/config/shiwai.yaml -p enable_motion:=false \
  > /tmp/rush_test.log 2>&1 || true
kill $FAKE 2>/dev/null || true
echo "=== 阶段切换日志 ==="
grep -E ">>>" /tmp/rush_test.log | sed "s/.*\[RUSH\] //" | uniq
echo
echo "期望顺序：对齐完成→视觉预告单边桥→IMU确认上桥→回平→已离桥"
echo "          →★入弯→★出弯→已停稳→★停车完成→对齐完成→★下蹲→★冲隧道→★冲出隧道"
echo "完整日志：/tmp/rush_test.log   假传感器日志：/tmp/fake.log"
