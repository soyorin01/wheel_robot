# 19 跳舞演示案例

本案例通过上位机 ROS2 节点发布 `/cmd_vel`、`/cmd_posture` 和可选 `/cmd_jump`，让轮腿机器人完成摆身、点头、滑步、原地慢转、下蹲起身、鞠躬归零等组合动作。

## 安全准备

1. 机器人放在平整地面，周围留出空间。
2. 如果遥控器可用，建议 `SWA` 打到 `2` 挡；没有遥控器时也可以运行本演示。
3. 先确认底盘桥接已经启动：

   ```bash
   source /home/orangepi/wheel_robot/install/setup.bash
   ros2 launch wheel_robot base.launch.py
   ```

4. 第一次先干跑，不让机器人运动：

   ```bash
   source /home/orangepi/wheel_robot/install/setup.bash
   ros2 launch wheel_robot_demos dance_demo.launch.py
   ```

5. 确认无异常后，再真实运行：

   ```bash
   source /home/orangepi/wheel_robot/install/setup.bash
   ros2 launch wheel_robot_demos dance_demo.launch.py dry_run:=false
   ```

## 调整动作

动作序列在 `config/dance_sequence.yaml` 中。常用字段如下：

- `height`: 腿长高度，默认限制 `0.20 ~ 0.28 m`
- `roll`: 机身横滚角，单位度，默认限制 `±6°`
- `pitch`: 机身俯仰角，单位度，默认限制 `±5°`
- `slide`: 滑步量，单位米，默认限制 `±0.03 m`
- `yaw`: 原地转弯角速度，单位 `rad/s`，默认限制 `±0.25 rad/s`
- `duration`: 平滑过渡到目标动作的时间
- `hold`: 到达目标动作后保持时间

想让动作慢一点：

```bash
ros2 launch wheel_robot_demos dance_demo.launch.py dry_run:=false speed_scale:=0.7
```

想循环两遍：

```bash
ros2 launch wheel_robot_demos dance_demo.launch.py dry_run:=false repeat:=2
```

## 保护逻辑

真实运行时，程序会检查：

- `/cmd_vel` 和 `/cmd_posture` 是否有下位机桥接订阅者
- `/robot_diagnostics` 是否正常
- 除 SBUS 遥控器相关故障外，是否存在 CAN、IMU、电机、电池等故障
- `robot_mode` 是否为 `0`

退出或异常时会连续发布零速度和默认姿态，尽量让机器人回到安全状态。没有遥控器时请在旁边准备断电，紧急情况直接切断机器人电源。
