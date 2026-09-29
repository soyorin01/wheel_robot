# shiwai.py 室外赛道调试手册

本文用于学生现场调试 `src/wheel_robot/scripts/shiwai.py`。比赛流程为：

```text
发车区 -> 单边桥 -> 弯道 -> 人行道停车 5 秒 -> 隧道下蹲通过 -> 起身 -> 终点停车
```

程序主要依靠普通摄像头识别黄线、黄黑胶带、人行道黑白条纹和终点黑字，并向底盘发布 `/cmd_vel`，向姿态节点发布 `/cmd_posture`。

## 1. 调试前先做这三步

1. 先只看画面，不让车动：

```bash
ros2 launch wheel_robot shiwai.launch.py enable_motion:=false
```

2. 确认画面方向正确，黄线、黄黑胶带、人行道、终点框能出现在调试窗口里。

3. 确认安全后再启用运动：

```bash
ros2 launch wheel_robot shiwai.launch.py enable_motion:=true
```

现场调车建议每次只改 1 个参数，小幅修改，跑一小段就停下复盘。

## 2. 参数改哪里

优先改 `src/wheel_robot/scripts/shiwai.py` 文件顶部的“现场调参”区域。程序启动后会用这些常量覆盖 YAML 里的部分速度、停车和下蹲参数。

常用开关和摄像头、基础识别阈值在 `src/wheel_robot/config/shiwai.yaml`。

不要优先改 YAML 里的这些项，因为会被 `shiwai.py` 顶部覆盖：

```text
forward_speed
minimum_forward_speed
max_angular_speed
curve_slowdown
bridge_speed_scale
bridge_climb_speed
bridge_climb_seconds
tunnel_speed_scale
sidewalk_stop_seconds
stand_height
crouch_height
height_rate
min_crouch_seconds
tunnel_pass_seconds
crosswalk_top_ratio
crosswalk_bottom_ratio
crosswalk_confirm_frames
min_crosswalk_stripes
```

## 3. 看调试画面和终端日志

调试窗口左上方会显示：

```text
RUN / SIDEWALK STOP / TUNNEL CROUCH / FINISH IN / FINISH STOP
err=横向误差  v=前进速度  w=转向角速度  h=腿高  roll=横滚
bridge=Y/N walk=Y/N tunnel=Y/N finish=Y/N
```

常见识别状态：

```text
tracking              正常识别黄线
no_yellow_lines       没识别到黄线
low_confidence        识别到了但可信度低
SIDEWALK STOP         人行道停车中
TUNNEL CROUCH         隧道下蹲通过中
FINISH IN             终点识别后继续驶入
FINISH STOP           终点停车
```

程序会自动保存识别录像：

```bash
ls -lt ~/wheel_robot/recordings/shiwai_*.avi | head
```

调不准时先看录像，不要只看车的结果。

## 4. 巡线不好怎么调

### 4.1 完全识别不到黄线

现象：

```text
调试窗口显示 no_yellow_lines
车不走或走一段就停
画面里明明有黄色泡沫，但掩膜区域很少
```

优先检查：

```text
摄像头是否对准赛道
画面是否过曝或太暗
黄色泡沫是否在搜索框内
```

可调参数：

```yaml
# src/wheel_robot/config/shiwai.yaml
yellow_h_min: 12
yellow_h_max: 42
yellow_s_min: 50
yellow_v_min: 70
min_confidence: 0.38
min_valid_row_fraction: 0.12
```

调整方法：

```text
黄色太浅或阴影下漏检：降低 yellow_s_min，例如 50 -> 40
画面偏暗漏检：降低 yellow_v_min，例如 70 -> 60
杂物也被认成黄线：提高 yellow_s_min 或 yellow_v_min
偶尔识别但可信度低：小幅降低 min_confidence，例如 0.38 -> 0.32
黄线只出现一小段：小幅降低 min_valid_row_fraction，例如 0.12 -> 0.08
```

### 4.2 车往反方向转

现象：

```text
黄线在右边，车应该右转却左转
黄线在左边，车应该左转却右转
```

修改：

```yaml
# src/wheel_robot/config/shiwai.yaml
steering_sign: -1.0
```

如果转向完全反了，把 `steering_sign` 改成相反数：

```text
-1.0 -> 1.0
1.0  -> -1.0
```

### 4.3 车直线左右摆动

现象：

```text
车能识别黄线，但左右来回晃
err 不大，但 w 变化很频繁
```

优先调：

```yaml
# src/wheel_robot/config/shiwai.yaml
steering_kp: 1.00
steering_kd: 0.025
error_filter_alpha: 0.45
```

调整方法：

```text
先降低 steering_kp，例如 1.00 -> 0.80
还抖动时小幅提高 steering_kd，例如 0.025 -> 0.035
识别点跳动明显时降低 error_filter_alpha，例如 0.45 -> 0.35
如果车反应太慢，再把 steering_kp 慢慢加回去
```

### 4.4 弯道转不过去

现象：

```text
大弯时冲出赛道
w 已经接近 max_angular_speed
速度太快，来不及转
```

优先改 `shiwai.py` 顶部：

```python
FORWARD_SPEED = 0.30
MINIMUM_FORWARD_SPEED = 0.20
MAX_ANGULAR_SPEED = 0.40
CURVE_SLOWDOWN = 0.60
```

调整方法：

```text
先降低 FORWARD_SPEED，例如 0.30 -> 0.24
再提高 CURVE_SLOWDOWN，例如 0.60 -> 0.75
仍然转不过去，小幅提高 MAX_ANGULAR_SPEED，例如 0.40 -> 0.45
```

### 4.5 车太慢或直线不够快

修改：

```python
FORWARD_SPEED = 0.30
MINIMUM_FORWARD_SPEED = 0.20
```

调整方法：

```text
整体太慢：提高 FORWARD_SPEED，例如 0.30 -> 0.35
弯道也太慢：提高 MINIMUM_FORWARD_SPEED，例如 0.20 -> 0.23
车身晃动或停车距离不准：先降速度，不要先加转向
```

## 5. 单边桥问题怎么调

单边桥由黄黑胶带触发。程序看到黄黑胶带后，会往右偏，让右轮上桥，并加速冲桥；过桥后会停车，再向左慢转找左侧海绵进入大弯。

### 5.1 看到单边桥太早，提前右偏

现象：

```text
车离桥还远就开始往右靠
日志很早打印“进入单边桥区域”
```

可调参数：

```yaml
# src/wheel_robot/config/shiwai.yaml
hazard_top_ratio: 0.08
hazard_bottom_ratio: 0.72
hazard_confirm_frames: 4.0
min_hazard_transitions: 1.8
```

调整方法：

```text
提高 hazard_top_ratio，让检测区域更靠近车，例如 0.08 -> 0.15
提高 hazard_confirm_frames，例如 4 -> 6，减少偶发误触发
提高 min_hazard_transitions，例如 1.8 -> 2.2，要求黄黑交替更明显
```

### 5.2 看到单边桥太晚，来不及上桥

现象：

```text
到桥头附近才开始右偏或加速
右轮没有及时对上桥面
```

调整方法：

```text
降低 hazard_top_ratio，例如 0.15 -> 0.08，让远处胶带也能触发
降低 hazard_confirm_frames，例如 6 -> 4
降低 min_hazard_transitions，例如 2.2 -> 1.8
```

### 5.3 右轮没上桥，从左侧滑下去

现象：

```text
车过桥时偏左
右轮没有压上桥面
```

优先改 `shiwai.py` 顶部：

```python
BRIDGE_OFFSET_CM = -10.0
BRIDGE_ROLL_DEG = 15.0
BRIDGE_CLIMB_SPEED = 0.50
```

调整方法：

```text
更往右走：把 BRIDGE_OFFSET_CM 改得更负，例如 -10 -> -12
右轮上桥力度不够：小幅提高 BRIDGE_CLIMB_SPEED，例如 0.50 -> 0.55
车身不够贴桥：小幅增大 BRIDGE_ROLL_DEG，例如 15 -> 17
```

### 5.4 车太靠右，撞右边或从右侧掉下

调整方法：

```text
把 BRIDGE_OFFSET_CM 改得没那么负，例如 -10 -> -8
降低 BRIDGE_ROLL_DEG，例如 15 -> 12
降低 BRIDGE_CLIMB_SPEED，例如 0.50 -> 0.45
```

### 5.5 冲桥动力不足，桥上卡住

修改：

```python
BRIDGE_CLIMB_SPEED = 0.50
BRIDGE_CLIMB_SECONDS = 7.0
BRIDGE_ON_RAMP_SPEED_SCALE = 1.50
```

调整方法：

```text
上桥瞬间没劲：提高 BRIDGE_CLIMB_SPEED
冲到一半就慢下来：增加 BRIDGE_CLIMB_SECONDS
已经在桥上但速度掉太多：提高 BRIDGE_ON_RAMP_SPEED_SCALE
```

注意：先确认机械结构和电量正常，再加速度。速度过大会导致冲出后面弯道。

### 5.6 过桥后还一直右偏，进弯压内圈

修改：

```python
BRIDGE_PASS_SECONDS = 3.2
BRIDGE_MIN_HOLD_SECONDS = 2.5
```

调整方法：

```text
右偏保持太久：减小 BRIDGE_PASS_SECONDS，例如 3.2 -> 2.8
很晚才结束单边桥状态：减小 BRIDGE_MIN_HOLD_SECONDS
```

### 5.7 还没过完桥就取消右偏

调整方法：

```text
增加 BRIDGE_PASS_SECONDS，例如 3.2 -> 3.6
增加 BRIDGE_MIN_HOLD_SECONDS，例如 2.5 -> 3.0
```

### 5.8 横滚一启动就歪，影响出发

修改：

```python
BRIDGE_ROLL_DELAY_SECONDS = 4.0
```

调整方法：

```text
刚出发就明显倾斜：增大 BRIDGE_ROLL_DELAY_SECONDS
到桥前才需要更早倾斜：减小 BRIDGE_ROLL_DELAY_SECONDS
```

## 6. 过桥后大弯问题怎么调

过桥后程序会先停车 `TURN_STOP_SECONDS`，再以 `TURN_LOST_ANGULAR` 向左慢转，直到锁到画面左侧的黄线。

### 6.1 过桥后不左转或左转太慢

修改：

```python
TURN_LOST_ANGULAR = 0.22
TURN_MIN_SPIN_SECONDS = 1.6
```

调整方法：

```text
左转太慢：提高 TURN_LOST_ANGULAR，例如 0.22 -> 0.26
还没看到左侧海绵就开始跟线：增加 TURN_MIN_SPIN_SECONDS
```

### 6.2 过桥后原地转太久

修改：

```python
TURN_LEFT_MAX_X_RATIO = 0.42
TURN_SINGLE_LEFT_HALF_RATIO = 0.22
```

调整方法：

```text
左侧海绵已经在画面左边但不锁：适当增大 TURN_LEFT_MAX_X_RATIO，例如 0.42 -> 0.46
锁左线后太贴左边：增大 TURN_SINGLE_LEFT_HALF_RATIO，例如 0.22 -> 0.26
```

### 6.3 过弯时贴左侧海绵

修改：

```python
TURN_SINGLE_LEFT_HALF_RATIO = 0.22
TURN_SPEED_SCALE = 0.55
```

调整方法：

```text
贴左侧海绵：增大 TURN_SINGLE_LEFT_HALF_RATIO
弯里速度太快：降低 TURN_SPEED_SCALE，例如 0.55 -> 0.45
```

### 6.4 过弯时往内圈偏

调整方法：

```text
减小 TURN_SINGLE_LEFT_HALF_RATIO，例如 0.22 -> 0.18
降低 TURN_LEFT_MAX_X_RATIO，避免把内圈黄块当左线
```

## 7. 人行道识别和停车怎么调

人行道由黑白条纹触发。识别到后程序进入 `SIDEWALK STOP`，先把速度降到接近 0，再开始计时 5 秒。

### 7.1 人行道识别太早，提前停车

修改 `shiwai.py` 顶部：

```python
CROSSWALK_TOP_RATIO = 0.72
CROSSWALK_BOTTOM_RATIO = 0.94
CROSSWALK_CONFIRM_FRAMES = 6.0
MIN_CROSSWALK_STRIPES = 5
```

调整方法：

```text
提高 CROSSWALK_TOP_RATIO，例如 0.72 -> 0.78，让条纹更靠近车才停车
提高 CROSSWALK_CONFIRM_FRAMES，例如 6 -> 8，减少误触发
提高 MIN_CROSSWALK_STRIPES，例如 5 -> 6，要求更多条纹
```

### 7.2 人行道识别太晚，压过停车区

调整方法：

```text
降低 CROSSWALK_TOP_RATIO，例如 0.72 -> 0.66，让更远处条纹也能触发
降低 CROSSWALK_CONFIRM_FRAMES，例如 6 -> 4
降低 MIN_CROSSWALK_STRIPES，例如 5 -> 4
降低 FORWARD_SPEED，给车更多制动距离
```

### 7.3 人行道完全不识别

先看调试窗口里 `CW 数字` 是否增加。`CW` 是检测到的条纹数。

可调：

```yaml
# src/wheel_robot/config/shiwai.yaml
black_value_max: 95
black_saturation_max: 110
```

调整方法：

```text
黑条太亮被漏掉：提高 black_value_max，例如 95 -> 110
有彩色阴影干扰：降低 black_saturation_max，例如 110 -> 90
条纹数不足：降低 MIN_CROSSWALK_STRIPES
```

### 7.4 其他黑色图案被误认为人行道

调整方法：

```text
降低 black_value_max，让只有更黑的条纹被识别
提高 MIN_CROSSWALK_STRIPES
提高 CROSSWALK_CONFIRM_FRAMES
提高 CROSSWALK_TOP_RATIO，让远处黑色干扰不触发停车
```

### 7.5 停车位置太靠前或太靠后

调整方法：

```text
停得太早：提高 CROSSWALK_TOP_RATIO
停得太晚：降低 CROSSWALK_TOP_RATIO，或降低 FORWARD_SPEED
停车后滑行太长：降低 FORWARD_SPEED / MINIMUM_FORWARD_SPEED
```

### 7.6 停 5 秒期间车还在摆动

程序只有在速度接近 0 后才开始计时。若停车期间晃动明显，优先降低速度和加速度。

可调：

```yaml
# src/wheel_robot/config/shiwai.yaml
max_linear_acceleration: 0.25
max_angular_acceleration: 1.00
```

```python
# src/wheel_robot/scripts/shiwai.py
FORWARD_SPEED = 0.30
MINIMUM_FORWARD_SPEED = 0.20
SIDEWALK_STOP_SECONDS = 5.0
```

调整方法：

```text
停车前冲击大：降低 FORWARD_SPEED 和 MINIMUM_FORWARD_SPEED
转向还在抖：降低 steering_kp
必须满足规则时，不要把 SIDEWALK_STOP_SECONDS 改小于 5.0
```

## 8. 隧道下蹲怎么调

人行道结束后，程序再次识别到黄黑胶带，会进入 `TUNNEL CROUCH`，发布较低腿高，通过隧道后恢复站立。

### 8.1 到隧道前没有下蹲

现象：

```text
过完人行道后，看到隧道入口胶带但状态没有变成 TUNNEL CROUCH
```

检查：

```text
walk 是否已经变成 Y
调试画面是否显示 HAZARD
黄黑胶带是否在 hazard 检测框内
```

调整方法：

```text
降低 hazard_top_ratio，让更远处胶带被看到
降低 min_hazard_transitions
降低 hazard_confirm_frames
确认人行道已经完成 5 秒停车，否则不会进入隧道流程
```

### 8.2 下蹲不明显

修改：

```python
STAND_HEIGHT = 0.25
CROUCH_HEIGHT = 0.16
HEIGHT_RATE = 0.20
```

调整方法：

```text
下蹲不够低：降低 CROUCH_HEIGHT，例如 0.16 -> 0.15
下蹲太慢：提高 HEIGHT_RATE，例如 0.20 -> 0.25
车身晃动明显：降低 HEIGHT_RATE
不要把 CROUCH_HEIGHT 调低于 0.14
```

### 8.3 在隧道里太早起身

修改：

```python
MIN_CROUCH_SECONDS = 4.0
TUNNEL_PASS_SECONDS = 0.1
MAX_CROUCH_SECONDS = 14.0
```

调整方法：

```text
刚进洞就起身：增加 MIN_CROUCH_SECONDS
胶带刚消失就起身：增加 TUNNEL_PASS_SECONDS
仍然太早：同时降低 TUNNEL_SPEED_SCALE，让通过更慢、更稳定
```

### 8.4 出隧道后一直不起身

调整方法：

```text
减小 TUNNEL_PASS_SECONDS
减小 MIN_CROUCH_SECONDS
确认 MAX_CROUCH_SECONDS 不要太大，必要时降到 10~12 秒
```

### 8.5 隧道里丢线停车

程序在 `TUNNEL CROUCH` 状态下丢线会慢速往前拱，不会立刻停。如果仍然容易停或走偏：

```python
TUNNEL_SPEED_SCALE = 2.0
TUNNEL_SINGLE_RIGHT_HALF_RATIO = 0.50
TUNNEL_SINGLE_LEFT_HALF_RATIO = 0.50
POST_TUNNEL_LOST_GRACE_SECONDS = 1.5
```

调整方法：

```text
隧道里贴右：增大 TUNNEL_SINGLE_RIGHT_HALF_RATIO，例如 0.50 -> 0.52
隧道里贴左：增大 TUNNEL_SINGLE_LEFT_HALF_RATIO，或减小右侧半宽
隧道速度太快：降低 TUNNEL_SPEED_SCALE
出洞口反光导致丢线停车：增加 POST_TUNNEL_LOST_GRACE_SECONDS
```

## 9. 终点识别和停车怎么调

终点是地上的黑色大字。程序只有在通过隧道并等待 `FINISH_AFTER_TUNNEL_SECONDS` 后才识别终点，避免把隧道口或人行道误判成终点。

### 9.1 终点识别太早

修改：

```python
FINISH_TOP_RATIO = 0.80
FINISH_BOTTOM_RATIO = 0.96
FINISH_AFTER_TUNNEL_SECONDS = 8.0
FINISH_CONFIRM_FRAMES = 6.0
```

调整方法：

```text
提高 FINISH_TOP_RATIO，让黑字更靠近车才触发
增加 FINISH_AFTER_TUNNEL_SECONDS
增加 FINISH_CONFIRM_FRAMES
```

### 9.2 到终点不停车

调整方法：

```text
降低 FINISH_TOP_RATIO，例如 0.80 -> 0.74
降低 FINISH_CONFIRM_FRAMES，例如 6 -> 4
降低 FINISH_AFTER_TUNNEL_SECONDS，前提是不会误判
检查调试画面是否显示 FN 和黑字框
```

### 9.3 停在终点前，没有驶入终点区

修改：

```python
FINISH_DRIVE_IN_SECONDS = 2.0
FINISH_DRIVE_IN_SPEED = 0.14
```

调整方法：

```text
停得太靠前：增加 FINISH_DRIVE_IN_SECONDS，例如 2.0 -> 2.4
或者小幅提高 FINISH_DRIVE_IN_SPEED，例如 0.14 -> 0.16
```

### 9.4 识别终点后冲过头

调整方法：

```text
减小 FINISH_DRIVE_IN_SECONDS
降低 FINISH_DRIVE_IN_SPEED
降低 FINISH_TOP_RATIO，让程序更早进入终点流程
```

## 10. 常见完整现象速查表

| 现象 | 优先看 | 优先改 |
| --- | --- | --- |
| 一启动不走 | `enable_motion`、底盘是否启动 | `enable_motion:=true` |
| 画面正常但不识别黄线 | 黄色掩膜、`no_yellow_lines` | `yellow_s_min`、`yellow_v_min`、`min_confidence` |
| 车反向转 | 黄线方向和车轮动作 | `steering_sign` |
| 直线左右摆 | `err` 和 `w` 抖动 | 降 `steering_kp`，加 `steering_kd` |
| 弯道冲出 | `w` 是否到上限 | 降 `FORWARD_SPEED`，加 `CURVE_SLOWDOWN` |
| 单边桥触发早 | 日志过早提示进桥 | 加 `hazard_top_ratio`、`hazard_confirm_frames` |
| 单边桥触发晚 | 到桥头才提示进桥 | 降 `hazard_top_ratio`、`hazard_confirm_frames` |
| 右轮不上桥 | 车身偏左 | `BRIDGE_OFFSET_CM` 更负，提高 `BRIDGE_CLIMB_SPEED` |
| 过桥后压内圈 | 右偏保持太久 | 减 `BRIDGE_PASS_SECONDS` |
| 过桥后一直转 | 左侧海绵不锁定 | 加 `TURN_LEFT_MAX_X_RATIO` |
| 人行道停早 | `CW HIT` 出现早 | 加 `CROSSWALK_TOP_RATIO` |
| 人行道停晚 | 压过条纹才停 | 降 `CROSSWALK_TOP_RATIO`，降速度 |
| 停 5 秒时晃 | `v/w` 未归零 | 降速度、降 `steering_kp` |
| 隧道不下蹲 | `walk=Y` 后是否有 `HAZARD` | 调黄黑胶带识别参数 |
| 隧道下蹲不明显 | `h` 是否接近 `CROUCH_HEIGHT` | 降 `CROUCH_HEIGHT` |
| 隧道里起身早 | `TUNNEL CROUCH` 时间短 | 加 `MIN_CROUCH_SECONDS`、`TUNNEL_PASS_SECONDS` |
| 终点不停 | 是否显示 `FN HIT` | 降 `FINISH_TOP_RATIO`、`FINISH_CONFIRM_FRAMES` |
| 终点冲过 | `FINISH IN` 时间太长 | 降 `FINISH_DRIVE_IN_SECONDS`、`FINISH_DRIVE_IN_SPEED` |

## 11. 推荐调试顺序

1. 摄像头画面：确认方向、亮度、黄线位置。
2. 黄线巡线：先让车稳定跑直线，再调弯道。
3. 单边桥：先调触发时机，再调右偏，再调冲桥速度。
4. 大弯：先确认过桥后能锁左侧海绵，再调弯道速度。
5. 人行道：先保证能识别，再调停车位置，再验证稳定 5 秒。
6. 隧道：先保证过完人行道后会下蹲，再调起身时机。
7. 终点：最后调终点识别和驶入距离。

## 12. 现场调参原则

```text
识别问题先看录像，不先改速度。
运动问题先降速度，不先加转向。
停车不准先降速度，再调识别框位置。
单边桥不过先调偏移，再调速度。
隧道问题先确认流程状态，再调腿高。
每次只改一个参数，改完重新启动 shiwai.launch.py。
```

