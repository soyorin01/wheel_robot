# ICN 调车手册

这份手册对应比赛主程序 `icn.py`。现场调车主要改：

```bash
src/wheel_robot/config/icn.yaml
```

不要优先改 `icn.py` 里的识别阈值。现在能完整跑下来，调车时每次只改一个参数，小步改，比如 `0.02m`、`0.03m/s`、`0.03rad/s`。

## 启动方式

先编译并刷新环境：

```bash
cd ~/wheel_robot
colcon build --packages-select wheel_robot
source install/setup.bash
```

如果底盘已经单独启动：

```bash
ros2 launch wheel_robot icn.launch.py
```

如果想一条命令同时启动底盘：

```bash
ros2 launch wheel_robot icn.launch.py start_base:=true
```

架空测试，不让车动：

```bash
ros2 launch wheel_robot icn.launch.py enable_motion:=false
```

比赛卡顿或不要 HDMI 调试画面：

```bash
ros2 launch wheel_robot icn.launch.py show_debug:=false
```

默认会同时启动 Astra 深度和 `step_detector_node.py`。如果你已经单独启动了台阶检测：

```bash
ros2 launch wheel_robot icn.launch.py start_step_support:=false
```

## 调车顺序

1. 先确认普通巡线能稳定跑，不压边、不频繁丢线。
2. 再调环岛：先保证第一圈，再看第二圈。
3. 再调台阶：先开 `manual_confirm_before_jump: true`，到起跳点停车后人工确认位置。
4. 最后调人行道停车和终点黄线。

## 普通巡线

位置：`drive`

| 现象 | 调哪个参数 | 怎么调 |
| --- | --- | --- |
| 直道太慢 | `forward_speed` | 加大 |
| 直道太快、容易冲 | `forward_speed` | 减小 |
| 弯道转不过来 | `max_turn_speed` | 加大 |
| 弯道转太猛、左右摆 | `max_turn_speed` | 减小 |
| 弯道冲出赛道 | `curve_slowdown` | 加大 |
| 弯道太慢 | `curve_slowdown` | 减小 |
| 90度大弯短暂看不到线就停 | `big_turn_loss_seconds` | 加大 |
| 90度大弯找线时冲出赛道 | `big_turn_loss_speed` | 减小 |
| 90度大弯找线转不过来 | `big_turn_loss_turn` | 加大 |

建议一次只改：

- `forward_speed`：每次 `0.02`
- `max_turn_speed`：每次 `0.03`
- `big_turn_loss_seconds`：每次 `0.10`

## 环岛

位置：`roundabout`

| 现象 | 调哪个参数 | 怎么调 |
| --- | --- | --- |
| 还没到环岛就触发 | `arm_after_start_distance` | 加大 |
| 到了环岛还不允许识别 | `arm_after_start_distance` | 减小 |
| 进环岛太早 | `first_lap_approach_distance` / `second_lap_approach_distance` | 加大 |
| 进环岛太晚 | `first_lap_approach_distance` / `second_lap_approach_distance` | 减小 |
| 转弯半径太大 | `left_turn_speed` | 加大 |
| 转弯半径太小、压内侧 | `left_turn_speed` | 减小 |
| 固定左转没进环岛 | `entry_turn_distance` | 加大 |
| 固定左转绕过头 | `entry_turn_distance` | 减小 |
| 进去了没出来 | `follow_max_seconds` | 减小 |
| 太早出环又拐回去 | `follow_max_seconds` 或 `exit_commit_seconds` | 加大 |

第一圈和第二圈可以分开调：

- 第一圈：`first_lap_approach_distance`
- 第二圈：`second_lap_approach_distance`

如果只有第二圈不准，优先只改第二圈参数。

## 台阶

位置：`step`

调台阶时建议先保持：

```yaml
manual_confirm_before_jump: true
```

这样到起跳点会先停车，终端按回车才跳。你可以先看车离台阶的位置是否合适，不合适就关程序改参数，避免一直摔车。

| 现象 | 调哪个参数 | 怎么调 |
| --- | --- | --- |
| 跳早了 | `sprint_distance` | 加大 |
| 跳晚了 | `sprint_distance` | 减小 |
| 稍微跳晚，需要提前触发 | `jump_trigger_lead` | 加大 |
| 稍微跳早，需要晚点触发 | `jump_trigger_lead` | 减小 |
| 冲刺速度不够 | `approach_speed` | 加大 |
| 跳不上去 | `jump_max_height` | 适当加大 |
| 弹太高、落地不稳 | `jump_max_height` | 减小 |
| 台阶太早接管巡线 | `arm_min_seek_distance` | 加大 |
| 一直不接管台阶 | `arm_min_seek_distance` | 减小 |
| 第二圈台阶前太快 | `second_lap_speed_cap` | 减小 |
| 跳完偏左找不回线 | `post_reacquire_turn` | 设为更负 |
| 跳完偏右找不回线 | `post_reacquire_turn` | 改成正数 |
| 跳完找线时间不够 | `post_reacquire_seconds` | 加大 |

台阶最常用只调两个：

- 跳早/跳晚：先调 `sprint_distance`
- 跳高/落地：再调 `jump_max_height`

## 人行道

位置：`crosswalk`

| 现象 | 调哪个参数 | 怎么调 |
| --- | --- | --- |
| 人行道压线/越过 | `approach_distance` | 减小 |
| 人行道停太早 | `approach_distance` | 加大 |
| 刚出环岛误识别人行道 | `detect_delay_after_roundabout` | 加大 |
| 人行道停留太久/太短 | `stop_seconds` | 对应调小/调大 |

## 起点黄线 / 终点

位置：`yellow_line`

| 现象 | 调哪个参数 | 怎么调 |
| --- | --- | --- |
| 第二圈看到黄线后停太早 | `finish_pass_distance` | 加大 |
| 第二圈看到黄线后冲太远 | `finish_pass_distance` | 减小 |
| 人行道刚结束误识别黄线 | `detect_delay_after_crosswalk` | 加大 |
| 只想跑一圈测试 | `total_laps` | 改成 `1` |

## 日志和录像

每次运行后，CSV 日志和录像会保存在：

```bash
src/wheel_robot/scripts/log
```

如果车表现异常，优先看终端最后打印的：

- `比赛CSV日志已保存: ...csv`
- `调试录像已保存: ...avi`

把这两个文件名记下来，再描述现象，才能判断是巡线、环岛、台阶检测还是里程计问题。

## 调参原则

- 能跑完整时，不要一次改很多参数。
- 第一圈好、第二圈不好，只调第二圈专用参数。
- 台阶摔车时，先打开人工确认跳跃。
- 识别画面明显但程序没触发时，先保存日志和录像，不要盲目调速度。
- 调完 YAML 后重新启动 launch，参数才会重新加载。
