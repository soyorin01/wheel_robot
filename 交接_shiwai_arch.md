# 交接文档 — shiwai_arch.py 巡线调车

> 给接手的新 AI / 新会话。先读完这份，再碰代码。
> 生成时间：2026-11-01 晚。源码 `src/wheel_robot/scripts/shiwai_arch.py`
> 5427 行，SHA256 前缀 `1f9c8bddb22faa41`。

---

## 0. 一句话背景

用户在户外调一台**轮足机器人**跑赛道（单边桥 → 大弯 → 人行道 → 隧道 → 终点）。
唯一有效的工作方式：**用户跑一趟 → 给录像或截图 → 我从 HUD/掩膜里量数据 → 改一两个参数 → 再跑**。
用户说话很短（"这为什么不停"、"又退回去了"），期望你**先查数据再下结论**，不要猜。

---

## 1. 项目与环境

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 22.04 aarch64（Orange Pi 4 Pro）|
| ROS | ROS 2 Humble |
| 工作区 | `/home/orangepi/wheel_robot` |
| 包 | `wheel_robot`(ament_cmake)、`wheeled_legged_pkg`(下位机) |
| 主脚本 | `src/wheel_robot/scripts/shiwai_arch.py` |
| 基线脚本 | `src/wheel_robot/scripts/shiwai_lin_line.py`（已知可用的老版本，4425 行）|
| 参数 | `src/wheel_robot/config/shiwai.yaml` |
| 录像 | `~/wheel_robot/recordings/shiwai_arch_YYYYMMDD_HHMMSS.avi` |

启动命令：

```bash
ros2 run wheel_robot shiwai_arch.py --ros-args \
  --params-file src/wheel_robot/config/shiwai.yaml -p enable_motion:=true
```

### ⚠️ 改完源码必须同步 install 副本

`shiwai_arch.py` 在 `CMakeLists.txt` 的 `install(PROGRAMS ...)` 里，`ros2 run` 跑的是
**install 里的副本**。曾经因此白查半天——用户以为在跑新代码，实际跑的是两小时前的 arch-v5。

```bash
cd ~/wheel_robot && cp src/wheel_robot/scripts/shiwai_arch.py \
  install/wheel_robot/lib/wheel_robot/shiwai_arch.py && \
  chmod +x install/wheel_robot/lib/wheel_robot/shiwai_arch.py && \
  sha256sum src/wheel_robot/scripts/shiwai_arch.py \
            install/wheel_robot/lib/wheel_robot/shiwai_arch.py
```

---

## 2. 硬件链与实测物理量

```
ESP32-S3 下位机 → /dev/ttyUSB0 → wl_base_node → /imu/data + /odom
                                 ↘ /cmd_vel, /joint_states
```

- `odom.twist.twist.linear.x = ROS_body.milemeter.Speed` —— **板载轮速计**，不是 ROS 算的。
  会滞后、会飘：车已停时 `ov` 还可能读到 ±0.2~0.5。
- **启动静摩擦阈值 ≈ 0.18 m/s**。指令 0.16 会原地打滑推不动（HUD 显示 0.16 但 `ov=0.00`）。
  这条已经害过三次（`TURN_WAIT_SPEED`、`BRIDGE_EXIT_SPEED`、`BRIDGE_SETTLE_SPEED`）。
- **双开 `wl_base_node` 会抢串口** → CRC 全错 → IMU 掉到 2 Hz、出现 1~5 s 空洞 →
  `len(samples)>=10` 永远不满足 → `base=None` → `bridge=fault`。
  排查命令：`pgrep -af wl_base_node`，正常只有 1 个，速率应 ~47 Hz。
- 腿高：站立 `0.20 m`，下蹲 `0.16 m`，俯仰 `PITCH_DEG=10°`。
- 轮距 43 cm，赛道内宽 84 cm，单边桥在右侧（`BRIDGE_OFFSET_CM = 5`）。

---

## 3. 调试录像怎么读（最重要的技能）

### 3.1 录像存的是**二值化后的 debug 图**，不是原始摄像头帧

`RECORD_DEBUG_VIDEO = True`，`_record_debug()` 写的是最终 `debug` 画布。
后果：**无法把录像重新喂给检测器**（白像素是掩膜，彩色是标注）。
需要复现检测器时，只能把近纯白像素重建成掩膜（会丢标注覆盖的那几像素，见 §5.4）。

格式：640×480，15 fps，MJPG。`帧号 = int(t*15)`。

### 3.2 颜色约定

| 颜色 | BGR | 含义 |
|---|---|---|
| 白 | (255,255,255) | 巡线掩膜（黄→白二值化结果）|
| 绿 | (0,220,0) | **左边线** L 的实测内沿 |
| 青 | (220,220,0) | **右边线** R 的实测内沿 |
| 橙 | (0,170,255) | PRED 预测/外推点 |
| 品红 | (235,40,235) | **CENTER 中线**（合成路径）|
| 白圈 | (255,255,255) r=8 | TARGET 目标点（= 加权中线中心）|
| 灰竖线 | (210,210,210) | 图像中轴 x=320（+ `TARGET_OFFSET_PIXELS`）|

### 3.3 HUD 布局（`_show()`）

```
y=26   state reason conf=
y=52   err=±0.000  v=0.00  ov=0.00  w=+0.00  h=0.20  roll=+0.0  pitch=+10.0
y=78   bridge=Y walk=Y tunnel=N finish=N  [fn-wait x.xs]  [post=xxx goal=0.00]  [extra]
y=94~138  ARCH 条（黑底）：
            ARCH {arch_pair_reason} side={boundary_side}
            CW {n} {HIT|-}  HZ={0|1}  FN={0|1}
y=110 / y=127   ← 被 ARCH 条盖住，pos=/dir=/width=、entry=/exit=/slope= 看不到
```

字段含义：
- `err` = `result.error`，控制量，**不是米**
- `v` = **指令速度**（不是实测）；`ov` = `/odom` 实测速度（我加的，只用于判断真实有没有动）
- `bridge=` 过桥时显示状态机 `crossing/exit_slow/exit_hold/done`，过完变成 `Y`
- `fn-wait x.xs` = 出隧道后"还差几秒才允许认终点"（`FINISH_AFTER_TUNNEL_SECONDS`）
- `post=exit-crawl goal=0.12` = 出弯后压速中

### 3.4 提取 v/w/ov 的工具

```bash
python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi --csv /tmp/vw.csv
python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi --start 8 --end 20
```
原理：HUD 格式固定、字符起点固定 → 用 `putText` 渲染模板做 `matchTemplate`。无 OCR 依赖。
CSV 列：`t,frame,v,w,ov,v_score,w_score`。

### 3.5 定位某一帧的三个套路

1. **按 HUD 数值**：跑 `hud_vw_series.py` 出 CSV，按 `v/ov/w` 过滤。
   例：`abs(v-0.46)<0.006 and abs(ov-0.69)<0.025 and w>0.05`。
2. **按图像特征**：算每帧巡线区（y 150~430）近纯白块的个数/形心 x、品红中线均值 x。
   例：找"左右各一大块 + 中线均值 x 在 140~220"。
3. **拼 HUD 条**（最高效）：把多个时刻的 `f[0:92].copy()` 竖着 `np.vstack` 成一张图，
   一次 `view_image` 看完十几个状态。**省 token，优先用这个。**

```python
cap=cv2.VideoCapture(path); want={237:'f237',243:'f243',245:'f245'}
rows=[]; i=0
while True:
    ok,f=cap.read()
    if not ok: break
    i+=1
    if i in want:
        g=f[0:92].copy()
        cv2.putText(g,want[i],(500,88),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),1)
        rows += [g, np.zeros((3,640,3),np.uint8)]
cv2.imwrite('/tmp/hud.png', np.vstack(rows))
```

---

## 4. 代码架构

### 4.1 类

```
DualYellowLineDetector          基类（与 shiwai_lin_line.py 相同）
  └─ ArchYellowLineDetector     出弯后启用 arch_* 掩膜清理/立柱剔除/双线配对
TrackSceneDetector              斑马线/胶带/终点识别
ArchSceneDetector               arch 版（arch_enabled 时接管）
ShiWaiFollower(Node)            主节点
```

`ArchYellowLineDetector` 的每个方法开头都是：

```python
if not self.arch_enabled:
    return super().<同名方法>(...)
```

覆盖的方法：`_curve_boundaries` / `_detect_columns` / `_curve_choose_single` /
`_yellow_mask` / `_stripe_exclusion_mask` / `_curve_fit_candidate` /
`_curve_fit_samples` / `detect`。

### 4.2 `arch_enabled` 什么时候开

`_update_bridge_target()` 末尾：

```python
enabled = (column_scan and _cleared_bridge and not _bridge.crossing
           and (arch_enabled or _curve_exit_ready or _passed_sidewalk
                or _mode in (MODE_STOP, MODE_CROUCH)))
```

即：**出弯（`_curve_exit_ready`）之后才开**，弯前/桥上/弯中一律走 lin 原逻辑。
`arch-v6` 的核心承诺就是"出弯之前和 `shiwai_lin_line.py` 一字不差"。

**已用 714 帧原始录像证明等价**（140852/144317/153928，含 199 帧 turn_entry、115 帧 turn_exit，
`valid/reason/error/confidence/rows/entry/exit` + debug 画面全部一致，差异 0 帧）。
改 arch 的代码后，如果涉及弯前分支，必须重跑这个等价性验证。

### 4.3 状态机

```
START → FOLLOW → [BRIDGE_APPROACH/CROSSING/EXIT_SLOW/EXIT_HOLD/SETTLE]
      → WAIT(出桥直行，至少 TURN_WAIT_MIN_SECONDS)
      → TURN(大弯，只跟左外侧海绵)
      → MODE_STOP(人行道停车 SIDEWALK_STOP_SECONDS)
      → MODE_CROUCH(隧道下蹲，TUNNEL_SPEED_SCALE=1.8 提速)
      → MODE_FOLLOW(出隧道)
      → MODE_FINISH(认出终点 → 拱入 → 自动退出节点)
```

### 4.4 `_control_cycle` 执行顺序（改代码前必看）

```python
_sync_bridge(now)                       # 桥状态机；fault 时发 0
相机可用/超时/重复帧检查                  # 早退
_update_bridge_target()                 # 切检测器参数档（站/蹲/弯）+ arch_enabled
result = _detect_lane(frame)
scene  = scene_detector.detect(frame, ..., prefer_finish=...)
_update_mission(scene, now)             # 状态机；_holding_stop() 时立即 return
_log_status(...)
if MODE_STOP:   _hold_sidewalk_stop(); return
if MODE_FINISH: _hold_finish_stop();   return     # ← 终点段只走这一条
if bridge fault / crossing / settle:  各自 return
_should_search_after_bridge → _handle_lost_line(searching=True); return
if not result.valid → _handle_lost_line(); return
arch 重获确认（仅 _passed_tunnel）
_follow_command(result, now)            # 转弯段走 _post_bridge_speed
```

### 4.5 代码里的坑

1. **`_detect_columns` 有两份完全相同的副本**（基类 ~1520 行 / arch ~3210 行）。
   同样的文本出现两次 → `replace_string_in_file` 会因歧义失败。
   办法：改常量，或用带区分上下文的 oldString，或分两次改（第一次改完后另一处就成了唯一匹配）。
2. `_log_status` 里还在用 `self.line_detector.arch_pair_reason`，**不能整体删除**。
3. `_curve_exit_ready` 是**锁存**的，到人行道停车后 `track_key` 变化才复位。
4. `_crouch_started` 只置一次、从不复位；`_passed_tunnel` 一旦 True 再也不回 False。
   所以"蹲一次、出一次"是**一局一次性**的。
5. `arch-v6` 重构时删掉了整套 `TURN_GAP_*` 常量 / `_turn_gap_*` 方法 / `turn_gap_untrusted`
   状态 / `_startup_can_move` / `_sensor_status`。不要再加回来。

---

## 5. 关键算法

### 5.1 中线误差是什么

`_detect_columns` 结尾：

```python
error = clip(lateral_error                       # 整条路径 x 的加权平均 vs 目标 x
             + CURVE_HEADING_GAIN(0.40) * heading_error
             + slope_term,
             -1, 1)
w = -1 * (STEERING_KP(0.72) * filtered_error + KD * d(error)/dt)
```

- `lateral_error = (Σ wᵢ·path_xᵢ − target_x) / (0.5·width)`，权重按近/中/远三段
  （`CURVE_STRAIGHT_WEIGHTS=(0.50,0.35,0.15)`，急弯渐变到 `(0.20,0.30,0.50)`）。
- `heading_error = sign(dx) · turn_strength · ramp`，
  `turn_strength = clip((路径角度 − 55°)/(85° − 55°), 0, 1)` —— **低于 55° 恒为 0**。
- `slope_term = clip(0.45 · slope_error, ±0.30)`，
  `slope_error = sign(dx) · min(|近端斜率| / 1.00, 1)`，
  只在 `_curve_exit_ready` 期间生效（**已取消 45 帧时限**）。

**已知缺陷（重要）**：`lateral_error` 是整条路径的平均值，一条斜穿目标的直线会平均成 0 →
控制器认为"正中"。所以必须靠 `slope_term` 表达"车身是斜的"。如果 `w=0` 而画面明显斜着，
先查这三项是不是同时为 0。

### 5.2 转弯段速度（`_post_bridge_speed`）

```python
target = max(TURN_MIN_SPEED,                                   # 0.23  ← 实际决定值
             FORWARD_SPEED(0.30) * TURN_SPEED_SCALE(0.55) * (1 - CURVE_SLOWDOWN(0.40)*turn_ratio))
```

第二项最大只有 0.165 < 0.23 → **`TURN_SPEED_SCALE` / `CURVE_SLOWDOWN` 在转弯段完全失效**，
只有 `TURN_MIN_SPEED` 和 `MAX_ANGULAR_SPEED` 有用。想快必须**两个一起提**，保持
$R = v/\omega$ 不变否则走线会切内：

| `MAX_ANGULAR_SPEED` | `TURN_MIN_SPEED` | 转弯耗时（实测 θ=151.5°）|
|---|---|---|
| 0.38 | 0.18 | 6.96 s |
| 0.48 | 0.23 | 5.51 s |
| 0.55 | 0.26 | 4.81 s |

### 5.3 单边推中线

```python
if expected_turn_direction:                 # 转弯段
    path = boundary + half[:,None] * normal          # 法向偏移，方向跟着边线
else:                                       # 平缓/隧道段
    path[:,0] += half  (left) / -= half  (right)     # 同一行水平偏移
```

`half = 0.5 · lane_width · ratio / 0.30`，`lane_width = 2·0.30·W·depth`，
`depth = (row/H − 0.10)/0.80`。

⚠️ 水平偏移模式下 **d(half)/dy = +0.83 px/row**，会主导合成中线的斜率，甚至和真实
车道方向相反（实测隧道里真实 −1.33，画出来 +0.54）。这是"单边中线很怪"的根源。

### 5.4 双边配对 `arch_pair_evidence`

```python
widths = right_inner − left_inner              # 同一图像行
usable = (widths >= min_lane_width_ratio*W) & (widths <= max_lane_width_ratio*W)
if usable 数量 < 24 或跨度 < 0.08H:  'width_or_span'
if mean(usable) < 0.60:             'inconsistent_width'
if slope(rows→widths) 不在 [-0.3, 4]: 'width_direction'
if 中位残差 > max(5, 0.025W):        'width_noise'
```

`max_lane_width_ratio` **按姿态切档**：站立/桥/弯 用 yaml 的 `0.82`，
`MODE_CROUCH` 用 `TUNNEL_MAX_LANE_WIDTH_RATIO = 0.98`（下蹲相机更低，车道在画面里更宽）。
HUD 的 `ARCH {reason}` 就是这里的 reason，`paired_measured` 才是真双边。

### 5.5 丢线处理（`_handle_lost_line`）—— 两套并存

```python
# ① 出隧道之后：ARCH 预算
if arch_enabled and _passed_tunnel:
    command = _arch_lost_command(now)
    # 预算: ARCH_LOST_SPEED_MAX=0.15, SECONDS=1.20, DISTANCE=0.30
    # 超了就 _publish_zero('arch_lost_budget') → "LOST STOP / WAIT STABLE LINE"
    return

# ② 隧道内（MODE_CROUCH）：TUNNEL CREEP —— ⚠️ 没有任何上限
if self._mode == MODE_CROUCH:
    creep_v = clip(|last_v|, MINIMUM_FORWARD_SPEED, FORWARD_SPEED*TUNNEL_SPEED_SCALE)  # ≤0.54
    ...  return
```

重获线要 `ARCH_RECOVER_FRAMES=3` 帧 + `ARCH_RECOVER_SECONDS=0.20` s。

---

## 6. 当前参数快照（2026-11-01 晚，用户手调后的实际值）

```python
# ---- 速度 ----
FORWARD_SPEED               = 0.30
MINIMUM_FORWARD_SPEED       = 0.20
MAX_ANGULAR_SPEED           = 0.55    # 用户从 0.38 一路加到 0.55
STEERING_KP                 = 0.72
CURVE_SLOWDOWN              = 0.40
TUNNEL_SPEED_SCALE          = 1.8     # 隧道巡航 0.36~0.54

# ---- 桥 / 大弯 ----
BRIDGE_EXIT_SPEED           = 0.20
BRIDGE_EXIT_HOLD_SPEED      = 0.20    # 不能降到 0.18 以下（静摩擦）
BRIDGE_SETTLE_SPEED         = 0.10    # ⚠️ 仍低于阈值，潜在打滑陷阱
TURN_WAIT_SPEED             = 0.20
TURN_MIN_SPEED              = 0.23
TURN_EXIT_SPEED             = 0.12    # ⚠️ 也低于阈值，实测 ov 只有 0.04
TURN_EXIT_SLOPE_GAIN        = 0.45
TURN_EXIT_SLOPE_REF         = 1.00
TURN_EXIT_SLOPE_MAX         = 0.30
TURN_EXIT_SLOPE_FRAMES      = 1 << 30 # 已取消时限

# ---- 巡线阈值（11-01 重定，压掉白噪点）----
LANE_YELLOW_H_MIN/MAX       = 20 / 40
LANE_YELLOW_S_MIN/V_MIN     = 45 / 90

# ---- 隧道 ----
STAND_HEIGHT                = 0.20
CROUCH_HEIGHT               = 0.16
MIN_CROUCH_SECONDS          = 2.0     # 用户刚改（原 4.0）
TUNNEL_PASS_SECONDS         = 0.1
MAX_CROUCH_SECONDS          = 14.0
TUNNEL_SINGLE_L/R_HALF_RATIO= 0.50 / 0.50
TUNNEL_MAX_LANE_WIDTH_RATIO = 0.98

# ---- 人行道 / 终点 ----
SIDEWALK_STOP_SECONDS       = 3.0
FINISH_CONFIRM_FRAMES       = 2
FINISH_AFTER_TUNNEL_SECONDS = 1.8
FINISH_IN_TUNNEL_MIN_SECONDS= 2.0
FINISH_DRIVE_IN_SECONDS     = 1.6     # 倒计时拱入时长
FINISH_DRIVE_IN_ACCEL       = 0.60    # 拱入加速斜率（绕开 _rate_limit 的 0.25）
FINISH_DRIVE_IN_SPEED       = 0.30
FINISH_STOP_HOLD_SECONDS    = 0.6     # 压零停稳后自动退出节点

# ---- ARCH 出隧道丢线预算 ----
ARCH_LOST_SPEED_MAX         = 0.15
ARCH_LOST_SECONDS           = 1.20
ARCH_LOST_DISTANCE          = 0.30
ARCH_RECOVER_SECONDS        = 0.20
ARCH_RECOVER_FRAMES         = 3
ARCH_VERSION                = "arch-v6"
```

---

## 7. 本会话（11-01）做过的改动 —— 都已落地并同步 install

1. **桥后不降到 0**：`BRIDGE_EXIT_HOLD_SPEED` 0→0.20；`BRIDGE_EXIT_SPEED`/`TURN_WAIT_SPEED` 抬到 0.20。
   根因：0.16 低于静摩擦，车滑停后再也起不来（145027 实测 dist 冻在 1.51 m）。
2. **HUD 加 `ov=`**：`v` 是指令不是实测，用户以为"虚假宣传"。
3. **黄线阈值 20/40/45/90**：白噪点/黄黑胶带混入的根因；144317 有效率 16%→48%，153928 79%→94%。
4. **斑马线掩膜改深青**：白色留给黄线，避免两套掩膜混淆。
5. **串口双开故障**：两个 `wl_base_node` 抢 `/dev/ttyUSB0` → IMU 断裂 → bridge fault。
6. **arch-v6 重构**：删掉全部 `TURN_GAP_*` / `turn_gap_untrusted` / `_startup_can_move` /
   `_sensor_status`；`_sync_bridge` 与 lin 逐字节相同；714 帧等价性验证 0 差异。
7. **隧道取消速度上限**：删掉 `ARCH_SINGLE_SPEED_MAX=0.18` / `ARCH_DUAL_SPEED_MAX=0.30`，
   隧道恢复 0.36~0.54（用户："隧道过的太慢了"）。
8. **终点倒计时固定速度**：`_hold_finish_stop` 拱入段不走 `_rate_limit` 的 0.25 加速，
   改用 `FINISH_DRIVE_IN_ACCEL=0.60`，否则 0→0.30 要 1.2 s 把倒计时耗光。
9. **重复帧不发 0**：`_control_cycle` 里 `sequence == _last_sequence` 时，
   `MODE_FINISH` 保持上一条指令（原来会砸一帧 0）。
10. **终点自动退出**：`FINISH_STOP_HOLD_SECONDS=0.6` + `_finish_exit_sent` +
    `request_exit("finish_done")`；`main()` 检测 `exit_requested`，`close()` 补发 10 帧零速度
    并关录像。
11. **出弯斜率项**：`TURN_EXIT_SLOPE_REF` 2.00→1.00（实测中位斜率 0.47），
    取消 45 帧时限（原来 3 s 后归零 → `err≈0`、`w=0`，车斜着横切赛道）。
12. **出隧道丢线预算放大**：0.08/0.35/0.04 → 0.15/1.20/0.30。
    原来 0.04 m ÷ 0.5 m/s = 80 ms ≈ 1 帧，一丢线就硬停 → 车往后溜 0.2 m → 再往前 → 重认终点。
13. **下蹲档宽度上限 0.98**：`arch_pair_evidence` 把真双边判成 `inconsistent_width`
    （近处宽度 627px > 0.82×640=525），退回单边 → 中线偏 190 px、`err=-0.495`。
    修后 `paired_measured`、`err=-0.038`。

---

## 8. 已知未解决问题（下一任的重点）

### 8.1 `TUNNEL CREEP` 完全没上限 ★用户已确认要改，尚未实施

`_handle_lost_line` 里 `if self._mode == MODE_CROUCH:` 直接 `return`，
`creep_v = clip(|last_v|, 0.20, 0.54)`，**没有时间也没有距离限制**。
实测（182400 录像 f237~f243）：黄线完全消失（`no_yellow_lines`、`conf=0.00`、`no_candidates`）
还在 0.53 m/s 盲冲 0.5 s，直到 `_passed_tunnel` 置位才切到 `ARCH_LOST_*`。
这次运气好它真出隧道了；若没置位就会一直冲。

建议改法（用户同意过方向，值可调）：

```python
TUNNEL_CREEP_MAX_SECONDS = 1.5   # 新增常量

if self._mode == self.MODE_CROUCH and lost_for < TUNNEL_CREEP_MAX_SECONDS:
    ... 现有 creep，return
# 超时后落到下面已有的 ARCH_LOST_* 预算（0.15 m/s / 1.2 s / 0.30 m）
```
⚠️ 注意 `if self._mode == MODE_CROUCH:` 这一支和上面的 `if arch_enabled and _passed_tunnel:`
是并列的，改成带条件的之后要确认超时分支真的能走到 ARCH 预算。

### 8.2 碎片关联的 8 px 绝对容差过严

`_curve_boundaries` 里（两处，约 1255 / 3250 行）：

```python
if abs(sa-sb) > 0.75 or abs(jump-0.5*(sa+sb)*gap) > 8.0:
    continue
```

162547 录像 f311 实测：最大白块 105×103（area 3860）与中段碎片本该连成一条泡沫，
`gap=21`、斜率 −1.25/−0.97 都合格，但 `x 跳变 −46` vs 外推 `−23`，差 23 px > 8 → 拒绝。
结果只用左下 38 行的小碎片合成中线。

建议：`tol = min(max(8.0, 1.2*gap), 24.0)`。已验算：能救回这一对，同时仍会拒绝
`#1↔#13`（差 53 px）避免过度合并。

### 8.3 单边合成中线的方向由先验主导（见 §5.3）

隧道里真实车道斜率 −1.33，画出来 +0.54（符号都反了）。根因是水平偏移模式下
`d(half)/dy=+0.83` 压倒了边线本身的斜率。如果要根治，得把平缓段也改成法向偏移，
但那会改变隧道/人行道的既有行为，需要重新标定 `TUNNEL_SINGLE_*_HALF_RATIO`。

### 8.4 `ov` 在 `v=0` 时跑到 −0.25（车真的往后溜）

180114 录像 f21.6~22.4 s 实测。指令 0 但轮速计读到 −0.07 → −0.25，约退后 0.2 m。
这是**底盘零速保持**的问题（`wl_base_node`），不在本脚本。若以后还会停，
需要查下位机的零速闭环。改完 §8.1 不再停车就不会触发。

### 8.5 两个速度常量低于静摩擦阈值

`BRIDGE_SETTLE_SPEED = 0.10`、`TURN_EXIT_SPEED = 0.12` 都 < 0.18。
`TURN_EXIT_SPEED` 已有实测证据：HUD `v=0.12` 但 `ov=0.04`（几乎不动）。
调到 0.20 更安全，代价是接近斑马线时快一点（斑马线本身有 6 帧确认 + 停车）。

### 8.6 未做过的实车验证

- 8.1 / 8.2 的改动**只做了离线数据推演，没上过车**
- 大弯 0.55/0.23 这一档只跑过一次，需要看有没有 `TURN LOST` / `TURN SEARCH`
- 终点自动退出只做过代码级模拟（2.0 s 后 `request_exit`），没跑过完整一圈

---

## 9. 工作纪律（都是踩过的坑）

1. **改之前必须核对文件状态**：用户/格式化工具会随时改文件。
   `sha256sum` + `wc -l` + `grep` 原文，确认你要改的那段还在。
2. **`py_compile` 抓不到跨类属性错误**。关键改动要用 `Fake` 对象调用真实方法验证：

```python
import sys; sys.path.insert(0,'.')
import shiwai_arch as M
F = M.ShiWaiFollower
class B: boost=False; distance=0.0
class Fake: pass
f = Fake(); f._bridge = B(); f.enable_motion=True; f.control_rate=15.0
f.max_linear_acceleration=0.25; f.max_angular_acceleration=1.0
f._approach = F._approach
f._publish = lambda l,a: f.published.append((l,a))
F._hold_finish_stop(f, None, now)     # 直接调未绑定方法
```
3. **不要"丢线后继续前进"**。用户明确反感在洞里冻住，但也反感完全没线还盲冲 —— 需要"有限预算"。
4. **改完立刻同步 install 副本**，否则 `ros2 run` 跑的是旧代码。
5. **终端 heredoc 长脚本的输出会被回显截断**。复杂脚本写 `/tmp/xxx.py` 再 `python3 /tmp/xxx.py`。
6. **回答要带数字**。用户是现场调车，只认量出来的数据（哪个录像是第几帧、err 从多少到多少）。
7. 单位：`err` 是归一化量（1.0 = 半幅画面 320px）；`pos` 同；`dist` 是桥区间轮式里程，不是全局坐标。

---

## 10. 常用命令

```bash
# 编译检查
cd ~/wheel_robot/src/wheel_robot/scripts && python3 -m py_compile shiwai_arch.py && echo OK

# 同步 install
cd ~/wheel_robot && cp src/wheel_robot/scripts/shiwai_arch.py \
  install/wheel_robot/lib/wheel_robot/shiwai_arch.py

# 提 HUD 数值
python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi --csv /tmp/vw.csv

# 查串口是否被双开
pgrep -af wl_base_node

# 对比 lin 与 arch 的方法差异
cd src/wheel_robot/scripts && python3 -c "
import ast,difflib
def m(p,c):
    s=open(p,encoding='utf-8').read(); t=ast.parse(s); L=s.splitlines()
    for n in ast.walk(t):
        if isinstance(n,ast.ClassDef) and n.name==c:
            return {x.name:'\n'.join(L[x.lineno-1:x.end_lineno]) for x in n.body if isinstance(x,ast.FunctionDef)}
a=m('shiwai_lin_line.py','ShiWaiFollower'); b=m('shiwai_arch.py','ShiWaiFollower')
for k in sorted(set(a)&set(b)):
    if a[k]!=b[k]: print(k, '不同')
"
```

---

## 11. 给下一任的一句话

**先要录像，量数据，再改一个数。** 这套代码已经被反复微调过几十轮，
每一个"奇怪"的参数背后都有一段实测注释；改之前先读那段注释和 `readme.md` 里对应日期的记录，
不要凭直觉调。
