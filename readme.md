# 杭电轮腿一队调试日志
起 2026.10.1 王赛
-----------------
# 必看（日志说明）
该文件存放基本的教程和调试日志记录
## 如何使用？
简单来说，就是存放你做了什么，改了什么，或者有什么需要交代的
调车的困难就在于参数和灵感，昨天的参数和灵感都是昨天的，不妨将它立刻写下来、记录下来，日积月累才能调好车，这也是我的习惯
所以，如果有任何文件的修改，新增或者删除，一定一定一定要在此文件记录下来，虽然有git，但此方式防遗忘，也好溯源

# AI代码说明
不建议AI参加日志的编写，另外不建议整个用AI调试
我不管你模型有多好，AI写的代码一定要review！或者足够调试，不然是绝对不会采用的！

# 连接小车
使用vscode ssh-remote
需要小车和电脑连接同一个局域网
目前暂时用热点：手机热点名字设置为robot，密码为123456789
输入命令：ssh orangepi@orangepi4pro.local
或，ssh orangepi@10.218.252.13

# git 仓库说明
仓库地址 https://github.com/soyorin01/wheel_robot
更新仓库
git add .
git commit -m "你想说的话"
git push

# 遥控器使用说明
模式	SWA	SWB	SWC	SWD	VRA	VRB	左摇杆	右摇杆
停止	0	0	任意	任意	最小	0	任意	任意
遥控模式1	1	0	1或者0	0	最大	0	前后与航向	高度与横滚
遥控模式2	1	0	1或者0	1	最大	0	前后与航向	高度与俯仰
ROS模式	2	接收到上位机数据控制移动底盘：前后，航向，横滚，俯仰，高度。上位机没有发布数据，遥控器会接管控制权。如果SWA是自动复位，SWA下拨动一次就是启动ROS控制模式
上电前，确保轮腿伸直成两条直线。确保遥控器处于初始位置
顺时针调节VRA旋钮，直到听见蜂鸣器
随后将SWA拨到1，将机器人放到地上即可

关机一定要放回架子上并把遥控器置初始位置！！

# 重新编译ESP32下位机
当前哈希值：e93a951b54ba9882fd2324134f9b406b37daa1fd589ef16a8967aa17993b59c9   ---10.9
跑通了，果然是官方固件的问题

# 键盘遥控机器人
1.确保SWA置于2
ros2 launch wheel_robot base.launch.py
ros2 run wheel_robot keyboard_teleop.py
/cmd_vel	Twist		前后 + 转向
/cmd_posture	JointState		腿高 / 横滚 / 俯仰 / 滑步
    posture.name = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
    posture.position = [self.height, self.roll, self.pitch, self.slide]
/cmd_jump	Bool		跳跃触发
# 摄像头节点astra_camera.launch.py
/camera/color/image_raw 彩色图 640×480@30
/camera/color/camera_info	CameraInfo	彩色内参
/camera/depth/image_raw	Image	深度图 16UC1（mm）
/camera/depth/camera_info	CameraInfo	深度内参
/camera/depth/left_distance	Range	左前障碍距离
/camera/depth/center_distance	Range	正前障碍距离
/camera/depth/right_distance	Range	右前障碍距离

# 首次测试dual_line_follower.py
视频在 /home/orangepi/wheel_robot/install/recordings/dual_white_line_20261009_153506.avi 
┌──────────────────────┬──────────────────┐
│                      │  ROI 320×160     │  ← 处理图（地面区域缩放后）
│   RAW 640×480        │                  │
│   （相机原图）        ├──────────────────┤
│   蓝框 = 地面 ROI     │  MASK 320×160    │  ← 白线二值掩码
│                      │                  │
│   ●  ●  ●            ├──────────────────┤
│                      │  遥测文字面板      │  ← FPS/error/state...
│                      │                  │
└──────────────────────┴──────────────────┘
颜色	形状	含义
🟢 绿色实心圆	半径 6（RAW 上 8）	NEAR 段（画面下部 / 离车最近）检出的白线质心
🟠 橙黄实心圆	同上	MID 段（中距离）检出的质心
🔵 天蓝实心圆	同上	FAR 段（画面上部 / 最远）检出的质心
🟣 品红空心圈	半径 8（RAW 上 10，线宽 3）	远中近加权融合后的最终目标点 target_x —— 这个才是真正喂给控制器去追踪的点
正常跟踪时：品红空心圈应该落在青色竖线上（见下）；如果偏了，车就会打方向去追。
颜色	形状	含义
⚪ 白色竖线	竖线	画面正中 x = 160
🩵 青色竖线	竖线	控制参考线 x = 125（= 160 + offset(−35)）—— 品红圈要收敛到这里
   彩色横线	横线	远/中/近三段的分界线（画在每段顶部 y0）。**三条天生都是横的**，其中 MID 那条是橙黄，最容易被误认成"黄线"
🟦 蓝色矩形框	框	RAW 上的地面 ROI 范围（上 45% 被裁掉）

修改了文件，加上了回放视频功能：
ros2 run wheel_robot line_follower.py --ros-args \
  -p video_source:=$HOME/wheel_robot/recordings/course_20261010_160202.avi \
  -p video_loop:=true -p show_debug_window:=true
# 注意：带 video_source 时代码会强制 enable_motion=False，不会驱动真车
# 录制视频
ros2 run wheel_robot camera_recorder.py
注意输出目录是 ~/wheel_robot/recordings/
已录制：
recordings/course_20261009_164310.avi
/home/orangepi/wheel_robot/recordings/course_20261009_164445.avi

# 10.10 丢线排查（line_follower.py 调参）
现象：丢线严重。用新增的 tools/inspect_frames.py 逐帧量出白线真实值：
    白线 S=50~125、V=110~175；红地面 S>=153；画面整体偏暗（V p50=71）
改动都在 line_follower.py 里（带注释）：

1. RED_S_MIN 65 -> 145          【主因】白线被红地面染成 H=5~13/S=50~125，被当成红色剔除了
2. WHITE_S_MAX 72 -> 135        白线 S 中位 75，原值砍掉一半以上
3. DYNAMIC_V_MIN 115 -> 90      原下限直接高于白线的 V，白线永远过不了
4. DYNAMIC_V_OFFSET 16 -> 8
5. DYNAMIC_V_PERCENTILE 82 -> 75
6. STRONG_WHITE_V_MARGIN 28 -> 12
7. MIN_VERTICAL_COVERAGE 0.24 -> 0.10
8. MIN_COMPONENT_AREA 20 -> 12
9. OPEN_ITERATIONS 1 -> 0（关闭开运算）、CLOSE_KERNEL_SIZE 5 -> 7
   3x3 开运算会把细白线侵蚀成碎片，是掩码「断点」的元凶

效果（75 帧，t=5~20s）：整帧有效 0% -> 91%，≥2段 0% -> 87%，≥3段 0% -> 39%

新增 tools/inspect_frames.py（离线调 HSV 用）：
    从录像截帧，导出【ROI | V | S | MASK】四联图 + 掩码联络表，
    直接复用 line_follower.py 的检测器和阈值。
    python3 src/wheel_robot/tools/inspect_frames.py recordings/xxx.avi \
      --start 8.0 --duration 0.3 --save-raw
    输出默认在 ~/wheel_robot/mask_debug/

调参证据图在 ~/wheel_robot/mask_debug/：
    morph_fix.png    开运算修正前后对比（最直观：碎片 -> 连续曲线）
    best_f282_t9.40s.png  最佳帧，3 段全中、置信度 0.956
    best_params.md   参数表 + 每项依据

# 10.10 dual_line_follower.py 同步调参
前 7 项改动与 line_follower.py 完全相同（RED_S_MIN / WHITE_S_MAX /
DYNAMIC_V_* / STRONG_WHITE_V_MARGIN / 关闭开运算 + 闭核改 7）。
另外必须多改一项：

    MAX_WHITE_PIXEL_RATIO 0.20 -> 0.50
    关闭开运算后掩码变厚（白像素中位 19%、最高 43%），原上限会否掉 26/75 帧，
    抬高后整帧有效 65% -> 100%
    （单线版实测这一项只 91%->92%，没必要改，故保持原值）

效果（75 帧，t=5~20s）：
    基线: 100%  /  ≥2段 49  /  ≥3段 18  /  conf 均值 0.382
    现在: 100%  /  ≥2段 75  /  ≥3段 72  /  conf 均值 0.900

# 10.11 dual_line_follower.py ROI_TOP_RATIO（看台干扰）

    ROI_TOP_RATIO 0.42 -> 0.54
    （这是模块常量，不是 ROS 参数，不能用 -p 覆盖，改完必须重新 colcon build）  

原因：画面顶部 0~52% 是看台（亮白低饱和）。它有两个害处：
  1. 被当成白线，而 FAR 段在融合里占 0.24 权重，会把方向盘带偏；
  2. 把 _make_mask 里整个 ROI 的全局动态亮度分位数抬上去，反而把真白线压掉。
  
dual_line_follower.py 也加了 video_source 回放（用法同单线版）：
    ros2 run wheel_robot dual_line_follower.py --ros-args \
      -p video_source:=$HOME/wheel_robot/recordings/course_20261010_160202.avi \
      -p video_loop:=true -p show_debug_window:=true

# dual_line_follower_illum.py
光照归一化，解决了局部过亮的问题
图像处理基本没什么问题，主要是导线逻辑不太行，建议忽略far
install/recordings_illum/dual_white_line_20261011_162200.avi

# shiwai.py
加入了日志输出，改了一些yaml里的参数
调试第一天
opencv处理有待优化
开局直接看到单边桥的胶带，就默认进单边桥模式导致平地起舞？
不应该消失而且过一小会再翘吗？
没搞懂，明天再修

# /home/orangepi/wheel_robot/src/wheel_robot/scripts/shiwai_far_line_fixed.py
① 巡线专用黄线阈值（分离出来了）
参数组	用途	H	S	V
lane_yellow_*（新增）	巡线（黄泡沫边缘）	24~40	≥35	≥70
yellow_*（保留原值）	场景识别（黄黑胶带 / 人行道 / 终点）	12~42	≥50	≥70
注册成了 ROS 参数，并在 _validate_parameters() 里加了范围校验。DualYellowLineDetector 现在吃 lane_yellow_*，TrackSceneDetector 还是吃 yellow_*。


② 巡线掩膜里排除「黄黑条纹」
新增 _stripe_exclusion_mask()（约 78 行），只在巡线掩膜里工作，不碰 TrackSceneDetector。

它替换掉了原来更粗暴的做法：

黄边旁边总有地面阴影 → 必然误删正常黄边。

新版要求局部重复的黄/黑交替才删：至少 4 段 run、跨度 ≤ 画面宽 16%、单段 ≥2px、未知色间隔 ≤4px，横竖两个方向都扫，最后按黄色连通块删（要求该块 ≥15% 面积被条纹支持）。

这个判据比「靠近黑色」合理得多。

③ 掩膜改成在整幅图上算
注释说明：条纹证据要在完整图像里确认，否则搜索框会把胶带裁成一小块黄色，判据失效。巡线 ROI 本身没扩大。
# shiwai.py
修改了单边巡线的虚拟中点

 0.38(=243px) 是估出来的，实测偏大：shiwai_20261018_142252.avi 第 0 帧
 双边 pairs=146、lane=397px，真实半宽只有 198px = 0.31。
 偏大意味着“虚拟中点”比真中点靠右 → 车会一直往右抖：
   左线在 x=100 时，0.38 → 虚拟中点 343 > 320 → 判成“该右转”，方向反了。
SINGLE_LEFT_HALF_RATIO = 0.31
SINGLE_RIGHT_HALF_RATIO = 0.31
BRIDGE_OFFSET_CM=0,取消了偏置带来的不必要的右边

增加了俯仰角10度，但还是很难看到赛道

# 修改巡线
位置	原	现	依据
SINGLE_LEFT_HALF_RATIO	0.24（154px）	0.30（192px）	实测半宽 190px
SINGLE_RIGHT_HALF_RATIO	0.24	0.30	同上
yaml expected_lane_width_ratio	0.49（314px）	0.59（378px）	实测车道宽 379px

勉强完赛18：00/home/orangepi/wheel_robot/recordings/shiwai_20261023_175631.avi
巡线偏左导致单边桥不稳，后面找单线找半天

# 10.23 晚 巡线分支 + 单边桥三段式
向下蹲改为2.2
## 新增文件
| 文件 | 作用 |
|---|---|
| scripts/shiwai_lane_fixed.py | shiwai.py 的改进分支（shiwai.py 本身一字未动） |
| launch/shiwai_lane.launch.py | 启动上面这个；节点名仍叫 shiwai，否则 yaml 的 shiwai 段不生效 |
| scripts/shiwai_posture_hold.py | 只发 /cmd_posture 保持低头，不占摄像头、不发车速 |
| scripts/shiwai_pitch_probe.py | 离线标定 err 偏置，自带 CSV+录像（占摄像头，别和 recorder 同跑） |

CMakeLists.txt 里补了对应的 install(PROGRAMS ...)。

## 单边桥改成三段式（中间插停车，方便看状态判断加速时机）
到桥头停 1.0s（BRIDGE_STOP_BEFORE_SECONDS）→ 冲桥 0.40m/s×2.0s → 再停 1.2s（BRIDGE_STOP_AFTER_SECONDS）
想取消停车就把这两个常量改成 0，别的地方不用动。

BRIDGE_LOCK_STRAIGHT=True：冲桥期间 angular 强制 0。
桥上只看得见一条泡沫，单线推虚拟中点会把车拐下桥（173117 掉桥时 err 已经到 -0.378）。

## 巡线
STEERING_KP 1.00 → 0.75（原来蛇行 ±20cm）
新增 TARGET_OFFSET_PIXELS：把车摆直道正中读到的静态 err × 320 填这里

## 转弯段找线（掉桥后找单线）
TURN_SEARCH_BOTTOM_RATIO = 0.94   泡沫在 y=375~410，原来 ROI 底 0.86 直接裁掉了
_turn_search_dir   按泡沫在画面哪一侧决定左转还是右转。
                   原来固定左转，泡沫在右侧时越转越远，永远满足不了 left_edge_x<269

---

# 10.25 `shiwai_lin_line.py`：曲线补线、桥上单边纠偏与出桥 WAIT 直行

本记录对应本次提供的 `shiwai_lin_line.py`，用于追加到仓库 `readme.md` 的调试日志中。以下参数按这份文件填写；本次整理文档没有修改源码，也没有上传仓库。

## 对比版本

- 核对日期：2026-10-25。
- 仓库：[`soyorin01/wheel_robot`](https://github.com/soyorin01/wheel_robot)。默认分支 `main`，核对时最新提交为 [`8c5f5c0`](https://github.com/soyorin01/wheel_robot/commit/8c5f5c0cb17d20485be279483f2deeaabc2770c1)，提交说明 `10.24`，时间 2026-10-24 19:59:30（北京时间）。
- 对比文件：仓库 [`src/wheel_robot/scripts/shiwai_lin_line.py`](https://github.com/soyorin01/wheel_robot/blob/8c5f5c0cb17d20485be279483f2deeaabc2770c1/src/wheel_robot/scripts/shiwai_lin_line.py) 与本次提供的同名文件。仓库版本 3451 行，本次文件 4071 行。
- 本次文件 SHA256：`ea4a53f43b04224fb7a387024113e3fbdaeddc036ab1f6d64c041e18abe61bc9`。

仓库最新 `lin` 文件已经具备 IMU/里程过桥状态机、桥后速度调整和可靠单边跟踪。本次新增的是曲线边沿处理、桥上单边视觉纠偏、入弯门槛及 WAIT 航向直行。README 前面关于“定时冲桥、角速度强制为 0”的内容是旧 `shiwai_lane_fixed.py` 分支的历史记录，不适用于这里的 `lin` 状态机。

## 与仓库最新源码相比改了什么

| 项目 | 仓库 `8c5f5c0` 的 `lin` | 本次文件 |
| --- | --- | --- |
| 边线提取 | 最长白列作为种子，逐行找内沿，再用直线补缺失行 | 从连通块提取连续内沿，使用鲁棒曲线拟合，适配近水平急弯 |
| 异常点处理 | 原有种子列、线宽及拟合约束 | 增加点级异常剔除、桥前持续横跳分段和近端实测支持检查 |
| 条纹排除 | 符合条纹重叠比例时删除整块黄色连通域 | 曲线检测路径只删除局部条纹支持区域，减少正常泡沫整块被误删 |
| 单边半宽 | 按阶段采用固定图像宽度比例 | 增加随图像行变化的半宽、桥前红线标定及可靠双边宽度学习 |
| 补线与置信度 | 拟合延伸行也计入边线可用行数 | 区分实测、剔除和补线点；限制缺口长度、外推和补点权重 |
| 急弯误差 | 以加权位置误差为主 | 增加沿路径的三段权重与方向补偿，结合唯一右弯所在阶段启用 |
| 桥上控制 | 保持航向，可靠双边提供小幅视觉修正 | 可靠单边也可持续纠偏；限制旧航向辅助对单边修正的抵消 |
| 出桥接管 | 确认离桥后交给桥后循迹 | 增加 `WAIT → TURN`；WAIT 保持出桥航向，接近弯口后恢复急弯循线 |
| 上桥目标速度 | `BRIDGE_CLIMB_SPEED = 0.40` | **`BRIDGE_CLIMB_SPEED = 0.60` m/s** |
| 调试信息 | 原有列扫描、边线统计和总误差 | 增加 `pos`、`dir`、`width`、`side`、`front`、`entry` 及桥上 `guide` |

`BridgeTraversal` 状态机、`TrackSceneDetector` 场景识别类和任务识别流程沿用仓库实现。此次没有新增场景识别降频或修改相机采集/控制频率，不能把其他分支的优化归入这份文件。

## 边线、补线和半宽

### 过滤点，保留当前帧误差

增加的过滤针对边界上的异常点、跳变片段和不合理候选，不是冻结突然变化的一整帧误差。原有控制器误差滤波仍保留。

桥前候选必须具有足够的纵向跨度，并延伸到近端实测区域。640×480 时，近端参考行为 y=288；需有至少约 48 个实测行、至少约 72 像素纵向跨度，以及近端至少约 10 个实测行，此外仍需通过曲线拟合本身的点数和跨度检查。补线点不能替代这些实测支持。

缺口最多补 24 像素；边缘外推最多 24 像素，同时不超过实测跨度约 10%。补点权重较低，且补点总权重不超过实测点权重的 1/4，即归一化总权重约 20%。允许关联同一条边的片段，不代表会把中间大段未知区域全部补满。

可靠单边线可以持续跟踪；补充边界身份关联和左线恢复确认。不能把“只有单边”当作已经丢线，但两侧都没有可靠检测时仍会进入相应的丢线处理。

### 桥前半宽标定

桥前默认半宽根据最后一次红线标注确定：同一图像行上，从左泡沫内沿量到期望赛道中线。公式为：

```text
half_width / image_width
    = 1.15716 × (image_row / image_height) − 0.32589
```

在 640×480、默认左右半宽比例 0.30 时，y=240/300/360 的半宽约为 162/254/347 像素。直接标注支持范围约 y=212～370，范围外属于外推，并受原有宽度限制约束。

可靠双边实测模型在其支持区域优先使用。当前阶段单边继续可见时，可沿用已经学到的宽度；分辨率或站立/蹲下宽度配置改变时重新建立模型。桥上仍走原检测路径，其半宽没有统一换成这条桥前标定曲线。

### 急弯权重

`CURVE_STRAIGHT_WEIGHTS = (0.50, 0.35, 0.15)`，`CURVE_SHARP_WEIGHTS = (0.20, 0.30, 0.50)`。三段沿路径距离划分，由近端到远端分别归一化，再结合实测/补点质量分配权重。

方向指标由实测边沿方向计算，55°开始增强、85°达到满值；这是图像方向指标，不是道路的物理转弯角。桥后到人行道前采用唯一右弯先验，仍允许车身偏右时进行左向纠偏。入弯确认前保留中线估计和弯道权重，但暂不加入方向增强。

## 桥上控制与速度

过桥状态仍为：

```text
idle → approach → crossing → exit_slow / exit_hold → done
异常时进入 fault
```

仍通过视觉预告、IMU 相对姿态和里程约束确认阶段，不按固定两秒强判离桥。此次桥状态机实现与仓库基准相同。

桥上可靠单边纠偏要求包括：置信度至少 0.85、主要一侧至少 40 行、另一侧点数不超过主要一侧的 25%。视觉修正限制为 ±0.10 rad/s，IMU 辅助不超过单边视觉修正幅度的一半，总角速度仍限制为 ±0.20 rad/s。可靠双边保留原有处理。

| 速度参数 | 本次值 | 作用 |
| --- | ---: | --- |
| `FORWARD_SPEED` | 0.30 m/s | 普通巡线速度，WAIT 直接共用 |
| `MINIMUM_FORWARD_SPEED` | 0.20 m/s | 普通循线速度下限，部分阶段另有缩放 |
| `BRIDGE_CLIMB_SPEED` | **0.60 m/s** | 上桥目标速度；仓库基准为 0.40 |
| `BRIDGE_ACCELERATION` | 0.60 m/s² | 桥区提速上限；仓库已有此值 |
| `BRIDGE_EXIT_SPEED` | 0.16 m/s | `exit_slow` 离桥确认速度上限 |
| `BRIDGE_LOST_LINE_SPEED` | 0.18 m/s | 桥上超过短时丢线保持期后的速度上限 |

目标速度还受状态和加减速限制影响；画面 `v` 是下发指令，不是实际地面速度。上桥速度改到 0.60 不代表完成了该速度下的实车通过验证。

## 出桥 WAIT：保持航向直行

完成离桥时记录一次当前 IMU 航向。WAIT 的角速度由“记录航向 − 当前航向”计算，不读取黄色边线的 `pos/error/dir`，也不运行循线 PID。参考航向不随每一帧重新设置。

WAIT 的速度直接取正常巡线的 `forward_speed`，当前为 0.30 m/s，并经过原有加减速限制。没有独立的 `TURN_ENTRY_SPEED` 低速上限。航向没有偏离时角速度目标为 0；偏离时用 IMU 小幅纠正。

图像仍用于确认何时入弯及原有可靠边线检查。尚未确认入弯时完全丢失可靠边线会停车，不原地扫线；IMU 超过 0.5 秒未更新或数据无效时也停车。恢复有效数据后可继续。

WAIT 没有固定秒数或固定直行米数。以下条件连续满足后进入 TURN：

1. 当前边线检测有效，并满足弯道方向条件。
2. 画面中央 30% 范围内有至少 24 个不同横坐标的实测点，横向跨度至少为整幅宽度的 10%。
3. 这些点的中位行达到图像高度的 0.90；640×480 下为 y=432。
4. 连续 3 次有效新帧检测满足条件；补线点不计入，空帧或不合格检测会打断计数。

进入 TURN 后恢复原有急弯控制，并保持入弯确认，避免转向过程中边沿在画面上移又退回 WAIT。WAIT 不积累循线 PID 历史，恢复循线时不带入此前远处黄线误差。

| 入弯/直行参数 | 本次值 | 调整作用 |
| --- | ---: | --- |
| `TURN_ENTRY_NEAR_ROW_RATIO` | 0.90 | 越大，要求实测弯沿在画面中越靠下才允许入弯 |
| `TURN_ENTRY_CONFIRM_FRAMES` | 3 | 连续有效新帧确认次数 |
| `TURN_ENTRY_CENTER_HALF_RATIO` | 0.15 | 中央检测区域为图像中轴左右各 15% 宽度 |
| `TURN_ENTRY_MIN_X_SPAN_RATIO` | 0.10 | 中央实测点的最小横向跨度比例 |
| `TURN_WAIT_HEADING_KP` | 1.0 | WAIT 的 IMU 航向保持增益，误差以弧度计算 |
| `TURN_ENTRY_MAX_W` | 0.12 rad/s | WAIT 航向纠偏上限，不是黄线纠偏上限 |

图像行门槛并不等于轮子距离泡沫的厘米数，相机姿态变化后需要重新核对。

## 调试显示与帧率

| 显示 | 含义 |
| --- | --- |
| `width=mark` | 桥前采用标注半宽先验 |
| `width=paired` | 已有双边实测宽度模型，覆盖外仍按原规则过渡 |
| `width=prior` | 其他曲线检测阶段使用一般宽度先验 |
| `front=Y` | 桥前近端实测支持检查已启用 |
| `pos` / `dir` | 位置项 / 实际用于误差计算的方向增强项；WAIT 的控制不采用视觉误差 |
| `entry=WAIT row=... need=432` | 当前中央实测行与入弯门槛 |
| `entry=TURN row=-` | 已确认入弯，不再重复计算入弯行门槛；横杠不表示丢线 |
| `post=entry-yaw goal=0.30` | WAIT 沿离桥航向直行，速度目标为正常巡线速度 |
| `post=entry-imu-stop` | WAIT 航向数据不可用，停车 |
| `guide=L/R/LR/IMU` | 桥上使用左线、右线、双线或 IMU 的控制依据 |

曲线调试图中，绿色为实测点，红色为剔除点，橙色为补点，青蓝色为推算中线路径。`CW` 框属于人行道场景检测显示，并不表示框中的白色都被巡线接受。

摄像头请求 30 fps、控制频率默认 15 Hz，沿用仓库配置。新图像到达后才重新检测；同一图像序号不会重复跑检测。实际图像处理、显示和录像耗时会影响有效处理频率；不能把相机请求帧率、录像声明帧率或离线耗时直接当成车载巡线帧率。

新增 WAIT 控制只做航向误差、限幅和速度变化处理，没有新增图像扫描。曲线检测、补线与点过滤属于相对仓库已有的图像算法变化；其整套开销仍需在车上测量。

## 替换和启动

将本次源码放到 `src/wheel_robot/scripts/shiwai_lin_line.py`。直接运行源码时可使用：

```bash
cd ~/wheel_robot
source install/setup.bash
python3 src/wheel_robot/scripts/shiwai_lin_line.py --ros-args \
  --params-file src/wheel_robot/config/shiwai.yaml \
  -p enable_motion:=true
```

实际发车时使用 `-p enable_motion:=true`，并沿用现有底盘、IMU 和里程话题连接。上述是启动命令说明，本次文档整理没有执行车辆启动。

需要核对入口：仓库最新 [`shiwai_lane.launch.py`](https://github.com/soyorin01/wheel_robot/blob/8c5f5c0cb17d20485be279483f2deeaabc2770c1/src/wheel_robot/launch/shiwai_lane.launch.py) 的 `executable` 仍为 `shiwai_lane_fixed.py`，且 [`CMakeLists.txt`](https://github.com/soyorin01/wheel_robot/blob/8c5f5c0cb17d20485be279483f2deeaabc2770c1/src/wheel_robot/CMakeLists.txt) 尚未把 `shiwai_lin_line.py` 列入 `install(PROGRAMS ...)`。因此只替换 lin 文件后，原 launch 不会自动改用它。若要通过该 launch 启动，需要另外调整 executable、安装列表并重新构建；本次没有修改这些文件。

启动提示应包含 `width-mark-v2`、`turn-entry-v1`、`wait-heading-v2`。

`_apply_field_tuning()` 会用源码顶部的场地常量覆盖对应 YAML 参数。因此速度、半宽、桥参数、WAIT 航向增益等以当前源码为准，修改后重启节点。`control_rate` 等未被覆盖的参数仍从配置加载。

## 本次核对范围

本次完成在线默认分支/提交核对、该提交原始文件提取、源码差异与语法树比较，以及提供文件的语法解析。

与上一份本地 WAIT 航向版相比，这份提供文件的类和方法实现相同，实际参数变化是 `BRIDGE_CLIMB_SPEED` 从 0.40 改为 0.60；`0.90` 写成 `0.9` 不改变入弯门槛。

本次没有运行 ROS 节点、发送运动命令或重新做实车测试。以前版本的离线测试数量和耗时不能直接记作本次 0.60 m/s 文件的验证结果；后续实测结果应另行追加到日志。

---

# 10.25 晚 大弯：入弯门槛 + 出弯判定

改动全在 `src/wheel_robot/scripts/shiwai_lin_line.py`。
**当前 4138 行，SHA256 `a78deabd8580b3ac30e36870756d274a9186c1f444b89d53e4a2e31e016209ad`**
（上面那份 4071 行 / `ea4a53f4…` 的记录已过时，别拿它当基准）。

## ① 入弯门槛 0.9 → 0.8

`TURN_ENTRY_NEAR_ROW_RATIO = 0.8`（HUD `need=432 → 384`）。

依据 `shiwai_20261025_181332.avi` 实测 row 轨迹（中央实测点中位行）：

```
8.4s=316  8.6s=335  8.8s=353  9.0s=377  9.2s=407  9.4s=445
```

- 0.9(432) 要等到 9.5s，那时 `pos` 已 +0.72、边界 0.4s 后就消失
- 0.5(240) 出桥第一帧就退，太早（试过，回滚）
- 0.8 让 3 帧确认正好在 **9.2s / dist=1.73m** 完成
- 7.4s 之前 row 只到 315，不会抢触发

## ② 新增出弯判定 `exit=`

位置：`_detect_columns` 里 `lateral_error` 算完之后。

```
box_x_max = 实测点中落在斑马线框(行 346~451)的最右横坐标
先 armed: box_x_max ≥ 0.75W
再判出弯: box_x_max < 0.50W  且  dir ≤ 0.60  且  |pos| ≤ 0.60
连续 3 帧 → exit=Y（锁死，直到人行道）
```

`|pos| ≤ 0.60` 只做粗筛，**故意放松**：出弯不触发时 pos 在弯里本来就可能到 0.4，
设成 0.2 会把 exit 永久卡死。

HUD 新增：`entry=TURN row=- need=384 exit=N slope=+0.00 bx=579`
其中 `bx` 就是 `box_x_max` 的真实值 —— 下次用它来定 0.50 这个阈值。

## ③ 新增出弯摆正 `slope=`

```python
err = pos + 0.40*dir + 0.45*slope_error     # slope 项只在 exit=Y 之后非零
```

`slope_error` = 近端中心线斜率 dx/dy 归一化。车摆正时斜率≈0，所以是**零偏置的
真误差**；而 `dir` 在 55° 以下完全瞎，补不上"位置对了但车身是歪的"这种情况。

## 实测记录

| 录像 | 现象 |
|---|---|
| `shiwai_20261025_181332.avi` | WAIT 里 row 315→445，9.5s 才 TURN，太晚 |
| `shiwai_20261025_190415.avi` | 能过弯；出弯后 w 骤减、车身没摆正 |
| `shiwai_20261025_191734.avi` | 过大弯不贴内测，另一个轮子没过去 |
| `shiwai_20261025_194528.avi` | exit=Y 在 14.2s 就触发（太早）；slope 一路饱和 +1.00 |

spole需要改，pos怪怪的
还有那个bx似乎没用，可以删
出弯判定太早（dist 2.53 < 弯道终点 ~2.8）
  → slope=+1.00（REF 0.45 太小，一上来就饱和）
  → err 里凭空 +0.45
  → pos 被压到 -0.45
  → 行驶线整体横向偏移
  → 黄泡沫不再落在 CW 框里 ← /home/orangepi/wheel_robot/recordings/shiwai_20261025_190415.avi这个turn是不错的，需要改

---

# 10.27 `shiwai_lin_line.py` 大弯 + 隧道 + 终点

**4303 行，SHA256 `6302cd521c5834f6eb047ffdae27170c3c53ca41bd6e132caf6fced0dceaea5c`**
（上面 4138 行 / `a78deabd…` 那份已作废）

验收：`shiwai_20261027_185159.avi` —— 大弯 → 出弯回正 → 斑马线停车 → 隧道，跑通。

## 当天改的参数

| 参数 | 旧 → 新 | 为什么 |
|---|---:|---|
| `TURN_WAIT_SPEED` | 0.30 → **0.16** | WAIT 冲太快，进弯时车身还在 0.30 |
| `TURN_MIN_SPEED` | 0.11 → **0.18** | 转弯半径 0.32m → 0.47m，不再原地打转 |
| `TURN_HEADING_RAMP_FRAMES` | 新增 **8** | 入弯首帧 `dir` 阶跃，`w` 撞限幅 → 丢线 |
| `TURN_ENTRY_NEAR_ROW_RATIO` | 0.75 → **0.70** | 旧值 row 差 6 行到不了就丢线 |
| 出弯 arming | `bx≥480` → **`dir≥0.60`，一次性** | 480 永远到不了，exit 永远 N |
| `TURN_EXIT_MAX_TURN_STRENGTH` | 0.60 → **0.15** | 车还在满量程硬转就判出弯 |
| `TURN_EXIT_MAX_LATERAL` | 0.60 → **0.35** | `pos=0.58` 时不该放行 |
| `TURN_EXIT_SLOPE_GAIN` / `_REF` | 0 / 0.45 → **0.45 / 2.00** | 出弯后唯一的航向项被乘 0 关掉了；实测斜率 1.0~2.2，旧尺子饱和 |
| `TURN_EXIT_SLOPE_MAX` / `_FRAMES` | 新增 **0.30 / 45** | 限幅 + 3s 归零，不许变永久偏置 |
| `TURN_EXIT_SPEED` | 新增 **0.12** | 出弯压速，别冲过斑马线 |
| `BRIDGE_SETTLE_*` | 新增 | 下桥先等车身稳住再进 WAIT |
| `TUNNEL_SPEED_SCALE` | 2.0 → **1.2** | 0.60 m/s 刹车距 0.70m > 终点带 0.28m |
| `FINISH_CONFIRM_FRAMES` | 6.0 → **3.0** | `FN` 稳定 HIT 只有 5 帧，6 帧凑不齐 |
| `FINISH_IN_TUNNEL_MIN_SECONDS` | 4.0 → **2.0** | 终点字在隧道末段就出，等 `_passed_tunnel` 晚 2.7s |

## 三个坑

1. **arming 必须 `if not self._curve_exit_armed:` 外层判断。**
   兜底条件写进 `if (... or frames>=45)` 会每帧清零计数器，exit 永远不落 Y。
2. **`FN 4` 不等于终点字。** 191729 24.3s 是隧道口胶带，192546 28.9s 才是真字，
   两次 `blobs` 都是 4 ——「下蹲满 N 秒」这道时间门必须留着。
3. **`TURN_EXIT_SLOPE_REF` 别设小。** 0.45 会饱和成常值偏置，`pos` 被压死。

## 还没解决

- `dir` 有 55° 死区，出弯后完全失明，现在靠 slope 顶
- `bx` 只剩 HUD，可以删
- 隧道模式 7.6s 偏长，卡在 `hazard_gone_frames = 12.0`
