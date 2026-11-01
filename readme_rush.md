# rush.py 速通版 · 现场操作说明

脚本：`src/wheel_robot/scripts/rush.py`（已加入 `CMakeLists.txt` 的 `install(PROGRAMS)`）
改动前备份：`backups/rush_before_20261101_185316/`

---

## 1. 启动（两个终端，和现在一样）

**终端 A —— 先起底盘：**
```bash
ros2 launch wheel_robot base.launch.py
```

**终端 B —— 再起速通程序（就这一条）：**
```bash
source /opt/ros/humble/setup.bash && source ~/wheel_robot/install/setup.bash
ros2 run wheel_robot rush.py --ros-args \
  --params-file ~/wheel_robot/src/wheel_robot/config/shiwai.yaml \
  -p enable_motion:=true
```

不想发车、只想看识别（车轮不动，安全）：把最后一段换成 `-p enable_motion:=false`。

节点名故意沿用 `shiwai`，这样现有的 `shiwai.yaml` 才会被套用；
脚本顶部的 RUSH 段参数会覆盖 yaml。

---

## 2. 流程顺序

```
摆正车 → 启动 → 起步巡线对齐(约1~2帧)
  → 全速直冲 0.60 m/s（w=0，固件自己保航向）
  → 单边桥（沿用原工程那套视觉+IMU 状态机，速度 0.60）
  → 桥后直冲
  → ★写死右转弯 v=0.60 w=-2.43（左轮 1.10 / 右轮 0.10）
  → ★出弯立即停车，静止满 5.0 秒（不看斑马线）
  → 巡线对齐 → 下蹲 0.16
  → ★全速盲冲隧道 2.6 秒
  → 起身直冲 → 认终点字 → 拱入 1.6 秒 → 停车 → 自动退出
```

终端里 `★` 开头的行就是阶段切换点，看日志就能知道车走到哪一步。

---

## 3. 参数改哪里

只改文件顶部的 `RUSH 速通现场调参` 段，改完保存、重启程序即可，**不用重新编译**。

| 常量 | 默认 | 说明 |
|---|---|---|
| `RUSH_SPEED` | 0.60 | 全程直冲速度（顶格 = 遥控最大前进速度） |
| `RUSH_BRIDGE_CLIMB_SPEED` | 0.60 | 冲桥速度。**真掉桥就先降到 0.50**（原工程实测值） |
| `RUSH_ALIGN_FRAMES` | 2 | 连续几帧有效线算「对齐完成」 |
| `RUSH_TURN_TRIGGER` | `"visual"` | 入弯触发：`"visual"` 沿用现有视觉确认 / `"odom"` 纯看里程 |
| `RUSH_TURN_START_DISTANCE` | 1.20 | 仅 `"odom"` 模式用，看下面标定日志填 |
| `RUSH_TURN_V` | 0.60 | 转弯前进速度 |
| `RUSH_TURN_W` | -2.43 | 右转为负。左轮≈1.10 / 右轮≈0.10 |
| `RUSH_TURN_YAW_DEG` | 90.0 | IMU 偏航转过这么多度就算转完 |
| `SIDEWALK_STANDSTILL_SECONDS` | 5.0 | 从「实测速度≈0」起算，保证规则要的静止 5 秒 |
| `RUSH_TUNNEL_SECONDS` | 2.60 | 下蹲后盲冲时长，**必须实测标定** |
| `RUSH_TUNNEL_SPEED` | 0.60 | 冲隧道速度 |
| `RUSH_MAX_DISTANCE` | 15.0 | 安全兜底：迟迟不进桥/不入弯就走这么远停车退出 |

### 转弯速度换算（左轮 / 右轮）

由 `/odom` 定义 `v=(L+R)/2`、`w=(R-L)/0.4118` 反推：`左=v-0.2059w`、`右=v+0.2059w`

| v | w | 左轮 | 右轮 | 转弯半径 |
|---|---|---|---|---|
| 0.23 | -0.48 | 0.33 | 0.13 | 0.48 m |（原工程实测）
| **0.60** | **-2.43** | **1.10** | **0.10** | **0.25 m** |（当前默认）
| 0.60 | -2.00 | 1.01 | 0.19 | 0.30 m |
| 0.60 | -1.50 | 0.91 | 0.29 | 0.40 m |
| 0.60 | -1.00 | 0.81 | 0.39 | 0.60 m |
| 0.60 | -3.00 | 1.22 | -0.02 | 0.20 m |（右轮开始倒转，包线极限）

`w` 越负转得越急。转不够就调更负，转过头就调回来。

---

## 4. 首次上车必须标定三件事

1. **转弯触发距离** —— 跑一趟，日志里会有
   ```
   [标定] 桥后里程 1.234 m（要改用 odom 触发就把 RUSH_TURN_START_DISTANCE 设成这个值附近）
   ```
   视觉触发没成功、或者想改成纯里程触发，就用这个数。

2. **隧道盲冲时长** —— 看录像量「车头进隧道到车尾出隧道」的时间，
   再留 0.3~0.5 秒余量填给 `RUSH_TUNNEL_SECONDS`。
   早起身会撞隧道顶（而且规则要罚「未起身」），晚起身只是慢一点，宁晚勿早。

3. **右转弯的 w** —— 按上面那张表，看车是转不够还是转过头。

---

## 5. 已知风险（照优先级）

1. **直冲段只靠固件保航向**。`w=0` 时固件会锁住「w 归零那一刻」的朝向，
   所以**起步必须自己把车摆正**；车头偏几度，直冲几米后就会偏出去。
   如果录像里发现明显跑偏，两个选择：改用 `RUSH_TURN_TRIGGER="odom"`（不依赖
   视觉，但也不修正横向偏差），或者在 `_rush_command` 里加一段 IMU 航向微调。
2. **入弯触发仍是视觉**，那套门槛是在 0.23 m/s 下调的，0.60 m/s 下能不能稳定
   触发，只能上车试。万一没触发，车会一路冲到头、到 `RUSH_MAX_DISTANCE` 停车
   退出（不会失控），日志里的「桥后里程」可以拿来切 `"odom"` 模式。
3. **隧道段完全不看路**，按你说的「自求多福」。
4. **桥速 0.60 没实测过**（原工程跑 0.50）。掉桥就降回去，掉桥罚 10 秒。

---

## 6. 看录像量速度

录像在 `~/wheel_robot/recordings/rush_日期_时间.avi`，同时有 `rush_latest.avi` 软链接。
HUD 第二行的字段顺序和原工程完全一致，所以原来的工具能直接用，**但 w 量程要调大**：

```bash
cd ~/wheel_robot
python3 src/wheel_robot/tools/hud_vw_series.py recordings/rush_latest.avi \
    --csv /tmp/rush_vw.csv --plot /tmp/rush_vw.png \
    --w-max 2.5 --w-limit 2.43
```
CSV 列：`t, frame, v, w, ov, v_score, w_score`（`ov` 是实测速度）。

不用这个参数的默认值（±0.60）会把转弯段的 w 削平。

---

## 7. 每轮 1 Hz 日志

```
[RUSH] RUSH  bridge=approach  v=+0.60 w=+0.00 ov=+0.58 左轮=+0.60 右轮=+0.60
       dist= 2.31m yaw=  +0.4 h=0.20 roll=+0.0 pitch=+0.0
       | 线 tracking conf=0.91 err=+0.012 entry=- | 场景 hz=0.00- fn=0.00/1-
       | 桥Y 道- 洞- 终- 夹=0
```
- `v/w` = 实际发出的指令，`ov` = 实测速度，`左轮/右轮` = 由指令反算的轮速
- `夹` = 被遥控轮速红线夹住的次数，**正常应该一直是 0**；不是 0 说明有参数超包线

---

## 8. 回滚

```bash
cd ~/wheel_robot
rm src/wheel_robot/scripts/rush.py install/wheel_robot/lib/wheel_robot/rush.py
cp backups/rush_before_20261101_185316/src/wheel_robot/CMakeLists.txt src/wheel_robot/
```
原工程 `shiwai_arch.py` 一个字都没动，随时可以直接跑回去。

---

## 9. 没有赛道时自测整条状态机

用假 IMU/odom 把 `ALIGN→RUSH→单边桥→WAIT_TURN→TURN→SIDEWALK→T_ALIGN→T_CROUCH→T_RUSH→POST_TUNNEL`
整条推一遍。**安全**：`enable_motion:=false`，不发任何运动指令；脚本还会先确认底盘节点没在跑。

```bash
bash ~/wheel_robot/rush_tests/run_state_machine_test.sh
```

期望顺序：
```
对齐完成 → 视觉预告单边桥 → IMU确认上桥 → 回平趋势 → 已离桥
  → ★入弯触发 → ★出弯 → 已停稳开始计时 → ★人行道停车完成
  → 对齐完成 → ★已下蹲 → ★盲冲隧道 → ★已冲出隧道
```

自测已验证通过（2026-11-01）。改完常量想确认没写崩，跑一遍这个就行，不用上车。
注意它只验状态机和时序，**不验视觉好不好用**——入弯触发、隧道时长、转弯角度这些
还是得在赛道上标定。
