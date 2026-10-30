#!/usr/bin/env python3
"""lin：批量图像检测、视觉预告单边桥、IMU/里程确认连续过桥。"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re
import math
from collections import deque
import signal
import subprocess
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


# ======================== 现场调参（优先改这里） ========================
# 改完保存后重新启动 shiwai 即可，不必改 yaml。
# 赛道两道黄泡沫内侧间距约 84 cm，轮足左右轮距 43 cm。
# 车走正中时：左轮在中线左侧 21.5 cm，右轮在中线右侧 21.5 cm。
# 单边桥在右侧，右轮会压上桥面，左轮仍在赛道地面，不要再往桥上偏。

TRACK_WIDTH_CM = 84.0
WHEEL_TRACK_CM = 43.0

# 过单边桥时相对黄泡沫中点的偏移（厘米）。
# 正数往左，负数往右。桥在右侧：右轮上桥、左轮在地面。
# 从左侧棱滑下去就把负数再加大（更往右），例如 -12。
BRIDGE_OFFSET_CM = 5 #-10，5

# 过单边桥时的横滚（度）。负数为左倾（左腿压低、右腿抬高），正数为右倾。
# 桥在右侧：左横滚让右轮更容易上到桥面。下桥后自动回到 0。
BRIDGE_ROLL_DEG =  0     #15
# 横滚变化速度（度/秒）。从 0 到 15° 大约 0.5 秒。
ROLL_RATE_DEG = 30.0
# 程序启动后这么多秒才允许左横滚，避免一出发就歪。神tm定时器
BRIDGE_ROLL_DELAY_SECONDS = 4.0

# ---------- 低头（俯仰 joint_pitching） ----------
# 摄像头看得太远就加大这个角度。全程保持，不随赛道阶段变化。
# 底盘单位是「度」，限幅 ±35°（wheeled_legged_pkg/src/wl_base_node.cpp: Pitch_limits=35）。
# 正负号：如果填 +5 反而抬头了，就改成 -5。
PITCH_DEG = 10.0
# 低头变化速度（度/秒）。从 0 到 5° 约 0.25 秒。
PITCH_RATE_DEG = 20.0

# 旧配置兼容项；lin 不再用“胶带消失 + 固定等待”判定上桥/离桥。
BRIDGE_ENGAGE_AFTER_GONE_SECONDS = 0.0
BRIDGE_ENGAGE_CONFIRM_FRAMES = 3.0
BRIDGE_ENGAGE_TIMEOUT_SECONDS = 12.0

# 只看到一侧黄线时，虚拟中点相对那条内沿的假定半宽（× 画面宽度）。
# 左：虚拟中点 = 左内沿 + 半宽。太小贴左，太大贴右。
# 右：虚拟中点 = 右内沿 − 半宽。太小贴右，太大贴左。
#
# ❗ 这个值直接由「相机离地高度 + 俯仰角」决定，改了 STAND_HEIGHT / PITCH_DEG
#    就必须重新标定，否则车会稳定地偏在一边（看起来像“横着调歪了”）。
# 实测 shiwai_20261023_172007.avi（h=0.20、pitch=10°）：
#   左右泡沫内沿量出车道宽中位 379px → 真半宽 190px = 0.30
#   旧值 0.24(=154px) 少了 36px → 虚拟中点偏左 → 车被稳到真中心左侧约 19cm。
SINGLE_LEFT_HALF_RATIO = 0.30
SINGLE_RIGHT_HALF_RATIO = 0.30
TUNNEL_SINGLE_LEFT_HALF_RATIO = 0.50
# 隧道里现在只能看到右侧海绵：半宽太小会贴右。还贴右就再加大，例如 0.52、0.55。
TUNNEL_SINGLE_RIGHT_HALF_RATIO = 0.50
# 过完单边桥进大弯：只顺着左侧外沿海绵走。
# 半宽太小贴左海绵，太大会往内圈偏。
TURN_SINGLE_LEFT_HALF_RATIO = 0.22
TURN_SINGLE_RIGHT_HALF_RATIO = 0.42

# 下桥后相对中点的偏移。大弯改跟左线，不再额外左拉。
TURN_OFFSET_CM = 0.0

# IMU/odom 确认离桥后有线直接跟随；丢线才搜索，不安排额外停车。
TURN_STOP_SECONDS = 0.0
TURN_MIN_SPIN_SECONDS = 0.0  # 兼容旧配置；不再强制旋转一段时间。
TURN_SEARCH_SECONDS = 5.0  # 首次没有可用边线时，最多旋转搜索五秒。
TURN_LOST_GRACE_SECONDS = 9.0
# 桥后丢线搜索目标：低速往前拱着找线，不再原地停转（于萧阳 10.29）。
TURN_LOST_SPEED = 0.10
TURN_LOST_ANGULAR = 0.22
# 锁左海绵时，其内沿必须在画面左侧。内圈黄块在右边，不能当左线。
TURN_LEFT_MAX_X_RATIO = 0.42
TURN_FALLBACK_MIN_CONFIDENCE = 0.70
TURN_FALLBACK_MIN_ROWS = 30
TURN_FALLBACK_MAX_ERROR = 0.95  # 饱和中心不作为可靠循迹依据。

# 桥后慢速模式的比例；稳定直线恢复 FORWARD_SPEED，不再整段固定乘 0.55。
TURN_SPEED_SCALE = 0.55
# 转弯段速度地板。旧值走 minimum_forward_speed*TURN_SPEED_SCALE=0.2*0.55=0.11，
# 170516/173351/174315 三次实测转弯段 v 全都卡在 0.11~0.14。
# 174315 实测 12.8~17.6s w 恒在 -0.34 rad/s、v=0.11，转弯半径 0.11/0.34=0.32m，
# 等于原地小圈打转（这正是「turn 基本都是原地在转」）。
# 抬到 0.18 后同样 w 下半径 0.18/0.34=0.53m，才是边前进边转。
#
# 0.18 → 0.23（提速一档）：转弯耗时 t = 转角/角速度，转角是赛道定的（170334
# 实测 148.5°），只能加角速度。v 必须跟着 ω 一起加，保持 R = v/ω 不変，
# 否则走线会变。旧 R = 0.18/0.38 = 0.474，新 R = 0.23/0.48 = 0.479。
TURN_MIN_SPEED = 0.23  #0.18->0.23（提速一档，要回退改回 0.18）
POST_BRIDGE_STABLE_SECONDS = 0.60
POST_BRIDGE_HISTORY_SECONDS = 0.80
POST_BRIDGE_MAX_SAMPLE_GAP = 0.25
POST_BRIDGE_MIN_SAMPLES = 5
POST_BRIDGE_MIN_CONFIDENCE = 0.75
POST_BRIDGE_MIN_LEFT_ROWS = 60
POST_BRIDGE_SMALL_ERROR = 0.12       # 归一化视觉误差，不是厘米。
POST_BRIDGE_SMALL_ANGULAR = 0.10     # rad/s；转向需求较大时立即请求降速。
POST_BRIDGE_MAX_ERROR_SPAN = 0.08   # 窗口内 max(error)-min(error)，防止左右抖动时加速。

# 过完桥允许用更远处的黄线重新锁线（0=搜索框顶，1=框底）。
TURN_NEAR_LOCK_RATIO = 0.08

# 转弯段专用的 ROI 底部（0=画面顶，1=画面底）。
# 低头 10° + 腿高 0.20 时相机只看得到脚前一小段，泡沫贴近车时会掉出默认的
# SEARCH_BOTTOM_RATIO=0.86（y=413）。实测 f126 泡沫已在 y=375~410，下一帧全空。
# 转弯段放宽到 0.94（y=451），把最近处那一段拉回搜索区。
TURN_SEARCH_BOTTOM_RATIO = 0.94

# ---------- 巡线搜索框（0=画面顶，1=画面底）。 ----------
# 框顶要能看到弯道远处黄线，框底要能看到脚下海绵。
SEARCH_TOP_RATIO = 0.24
SEARCH_BOTTOM_RATIO = 0.86
# 搜索框内：只有靠近车的这一段允许“新锁线”（0=框顶远处，1=框底近处）。
NEAR_LOCK_RATIO = 0.35

# ---------- 最长白列边缘提取（参考智能车常用算法） ----------
# 做法：先统计每列从画面底部向上的连续白点数，取“最左最长列”和“最右最长列”
# 作为左/右两条线的种子，再自底向上逐行从种子列向外找白→黑跳变得到边界。
# 丢失的行用已检出行的最小二乘直线外推（远端 → 近端），避免 error 处的
# 边线不明显时中心跳变。仅在「还没过单边桥」阶段启用。
COLUMN_FIT_MIN_POINTS = 8      # 至少要这么多行才拟合
COLUMN_FIT_MAX_ROWS = 140      # 离最近已检出行超过这么多行就不再外推
COLUMN_FIT_MAX_SLOPE = 1.5     # px/row，防止远端外推飞掉

# 最长白列 + 拟合延伸只在「非冲桥」阶段启用：
#   进桥前巡线 ✓   冲桥(crossing/exit_slow/exit_hold) ✗ 保持原有算法
#   下桥后大弯 ✓
# 冲桥那一段一行都不能动，关掉开关就完全退回原来的逐行 run 配对算法。
COLUMN_SCAN_ENABLED = True

# 连续内沿与急弯控制；权重按近、中、远排列，方向项在误差滤波前加入。
CURVE_BOTTOM_RATIO = 0.98
CURVE_MIN_POINTS = 24
CURVE_MIN_SPAN_RATIO = 0.12
CURVE_MAX_GAP_PIXELS = 24
CURVE_ASSOCIATION_GAP_RATIO = 0.15
CURVE_MAX_FIT_RMS = 8.0
CURVE_FILL_WEIGHT = 0.25
CURVE_FILL_TOTAL_RATIO = 0.25
CURVE_STRAIGHT_WEIGHTS = (0.50, 0.35, 0.15)
CURVE_SHARP_WEIGHTS = (0.20, 0.30, 0.50)
CURVE_HEADING_START_DEG = 55.0
CURVE_HEADING_FULL_DEG = 85.0
CURVE_HEADING_GAIN = 0.40
# 方向项不能阶跃打开。174315 实测 9.6s dir=+0.00、10.0s 入弯瞬间 dir 直接跳到
# +0.86，err 从 0.157 蹦到 0.826，w 立刻饱和到 -0.38，v 同时掉到 0.11，
# 0.4s 后（10.4s）就丢线开始原地转 1.4s。
# 所以入弯后让方向项在 TURN_HEADING_RAMP_FRAMES 帧内从 0 线性升到满量程。
# 15fps 下 8 帧≈0.53s，w 会从 0.12 平滑升到 0.36 而不是一帧跳到 0.38。
TURN_HEADING_RAMP_FRAMES = 8
# 方向项（"劲"）的释放加一阶惯性，不逐帧等于瞬时值。
# 1827 圈实测：出弯段 dir 0.9→0.7→0 只用了两帧，紧接着左侧泡沫出框丢线 2 秒，
# 而 lost_hold 重发的"上一条指令"此时已经是 err≈0 的直行 —— 车就笔直滑出去。
# 乘以这个系数让它按时间常数朝瞬时值收敛（15fps 下 0.08≈0.8s）：
# 永远在收敛，所以不是锁死，是动态释放（于萧阳 10.29）。
TURN_FF_LAG = 0.08
# 出桥看见弯道不等于车轮已到弯口：WAIT 保持出桥航向直行，不使用黄线误差。
# 近处条件只看画面中央 30% 内的实测点，不能用补点或近处的侧边端点触发。
# 181332 录像实测 row 轨迹（15 fps，这段约 10 px/帧）：8.4s=316 8.6s=335 8.8s=353
# 9.0s=377 9.2s=407 9.4s=445。旧的 0.9(432) 要等到 9.5s，那时 pos 已 +0.72、
# 边界 0.4 s 后就消失；0.5(240) 出桥第一帧就退太早；0.7(336) 也还偏早。
# 实测合适的入弯点是泡沫进入人行道检测框（CROSSWALK_TOP_RATIO=0.72，第 346 行）
# 之后再走约两帧。20261027 两次实测：
#   170516：到 row=390 才入弯，太晚，入弯后 0.8s 泡沫就被甩出画面丢了线
#   173351：用 0.75(=360) 时 row 最高只到 354，差 6 行没到 360，然后又丢线
# 故取 0.70(=336)：173351 约 8.9s 就能入弯，赶在 10s 丢线之前。
# 0.5(240) 已验证是明显太早。
TURN_ENTRY_NEAR_ROW_RATIO = 0.70  # 480 高时 need=336。
TURN_ENTRY_CENTER_HALF_RATIO = 0.15
TURN_ENTRY_MIN_X_SPAN_RATIO = 0.10
TURN_ENTRY_CONFIRM_FRAMES = 3
TURN_ENTRY_MAX_W = 0.12  # rad/s，仅限制 WAIT 的 IMU 航向纠偏，不再读取黄线误差。
TURN_WAIT_HEADING_KP = 1.0  # 航向误差用弧度。
# WAIT 段前进速度。原来直接复用 forward_speed(0.30)，太大：
# 174315 实测 7.6s 出稳定、9.6s 就 row=352 触发入弯，2.2s 内冲了 0.36m，
# 车身带着 0.30 的速度一头撞进弯口，入弯后 0.4s 泡沫就被甩出画面。
#
# 但 0.16 又太小了：221612 实测 5.2~8.5s 指令一直 0.16，
# 里程计 6.0s→8.0s 只涨了 0.04m（≈0.02 m/s），车在桥下原地打滑推不动，
# 而 HUD 画的是指令值所以一直显示 0.16 —— 看着像停住。
# 同一趟 15~17.5s 用 0.12 在平地上里程涨了 0.32m，说明不是速度死区，
# 是桥下那一段需要更大的劲儿才能从静止推起来（入弯用的 0.23 实测能推动）。
TURN_WAIT_SPEED = 0.24  # 0.16 -> 0.24，前提是桥下那段能一直滚着不起来。
# WAIT 至少跑这么久才允许入弯。_cleared_at 是离开 BRIDGE SETTLE、真正开始
# WAIT 的时刻；没有这道门时，只要 row 一过线就立刻 TURN，车身还没摆正就进弯。
# 184532 实测：10.0s 刚出稳定、10.4s 就 entry=TURN，WAIT 只跑了 0.4s。
TURN_WAIT_MIN_SECONDS = 1.8 #0.8
# ---- 出弯判定 + 出弯摆正 ----
# 旧方案靠「框内最右端 bx」两步：先 bx>=0.75W(=480) 置 armed，再 bx<0.50W(=320)
# 才计数。实测这个 arming 根本到不了，四条录像 bx 峰值：
#   173351: 375   174315: 243   175158: 379   175657: 309
# 全部 < 480，所以 exit 永远是 N。175657 跑到 17.6~19.2s 已经 dir=+0.00、
# pos=+0.04~+0.19（弯早就走完了）还在 TURN，结果 19.6s 又被判成 cruise
# 直接加速到 v=0.30 冲出去。
# 改成用方向项自己当 arming：真的进过弯（strength 到过 0.6）才允许判出弯。
# bx 不再参与判定，只留 HUD 观察。
TURN_EXIT_ARM_STRENGTH = 0.60   # 方向项曾经到过这个强度才算「进过弯」（一次性）
TURN_EXIT_ARM_FRAMES = 45       # 兜底：弯里磨了 3s 还没 armed 就直接置上，防卡死
TURN_EXIT_CONFIRM_FRAMES = 3
# 194528 录像 14.2s 泡沫还没完全出框就判成 Y（pos=+0.17 dir=+0.73），
# 而你要的 15.0s 是 pos=-0.00 dir=+0.54。所以再加两道 HUD 可见的守卫。
# 方向项跑到 0.15 以下才算弯走完，四条录像实测：
#   175158: 14.8s=0.53 15.6s=0.36 16.0s=0.18 16.4s=0.10 16.8s=0.00
#   175657: 15.6s=0.55 16.0s=0.49 16.4s=0.32 16.8s=0.15 17.2s=0.00
# 0.15 会在 16.4~16.8s 落 Y，用户点名的是 17.0s，正好。
TURN_EXIT_MAX_TURN_STRENGTH = 0.15   # 方向项必须已衰到这个值以下
TURN_EXIT_MAX_LATERAL = 0.35         # pos 也要收敛到中线附近。0.6 太松：
                                     # 175657 在 12.0s pos=+0.58、w 还在 -0.38
                                     # 满量程，那时过这道门就是误判。
# 出弯后用近端中心线斜率当航向误差把车身摆正：
# 车对准直道时斜率≈ 0，所以它是零偏置的真误差，不像 turn_strength 那样是常值偏置。
#
# 为什么必须打开它（183635 录像实测）：出弯后 err 只剩 pos 一个 P 项，
# 因为 dir 有 55° 死区（16.8s 后恒为 +0.00），而 slope 被乘 0 丢掉了。
# 纯 P 的横向项不能让车摆正，只能让它绕着线画圈：
#   16.4s exit=Y 后 w 一直 -0.11~-0.15 持续 3s（差 0.30rad≈17°），
#   pos 在 0.03~0.22 来回晃不收敛，末端 24.0s 直接跑成 pos=+0.48。
#
# 为什么要改 REF：从 183635 录像里量出近端真实 |slope| = 1.0~2.2 px/row，
# 旧的 0.45 让它 2~5 倍饱和，恒为 +1.00 —— 这正是 194528 那次把 pos
# 压到 -0.45 的原因。不是斜率错，是 REF 尺子错了。
TURN_EXIT_SLOPE_GAIN = 0.45
TURN_EXIT_SLOPE_REF = 2.00   # px/row，按实测 1.0~2.2 取满量程
TURN_EXIT_SLOPE_MAX = 0.30   # 斜率项对 err 的最大贡献，防出弯那一帧阶跃到 0.45
TURN_EXIT_SLOPE_FRAMES = 45  # 3s 后自动归零，绝不允许变成永久横向偏置
# 出弯后限速：斑马线就在前方，若让 _post_bridge_speed 判成 stable 会直接加速到
# forward_speed(0.30)，一路冲过人行道。173351 实测出弯后 4s 内 v 从 0.14 涨到 0.30，
# 然后 dist 4.20m 仍在下 CRUISE，全程没停。
# 不用 v=0：停在斑马线前会让 _detect_crosswalk 看不到条纹、永远不触发停车。
TURN_EXIT_SPEED = 0.12
# 近处半宽先验的参考行；真正宽度只从同帧双边实测更新。
LANE_HALF_RATIO_REFERENCE = 0.30
LANE_WIDTH_HORIZON_RATIO = 0.10
LANE_WIDTH_REFERENCE_ROW_RATIO = 0.90
LANE_WIDTH_MIN_ROWS = 24
LANE_WIDTH_MIN_SPAN_RATIO = 0.15
LANE_WIDTH_BLEND = 0.20
LANE_WIDTH_EXTRAPOLATION_RATIO = 0.12
LANE_SIDE_MATCH_RATIO = 0.06
LANE_LEFT_RETURN_FRAMES = 3
# 只用于桥前选线；阈值按整幅图像高度计算，不能由补线点满足。
PRE_BRIDGE_NEAR_ROW_RATIO = 0.60
PRE_BRIDGE_MIN_NEAR_ROWS_RATIO = 0.02
PRE_BRIDGE_MIN_ROWS_RATIO = 0.10
PRE_BRIDGE_MIN_Y_SPAN_RATIO = 0.15
PRE_BRIDGE_MAX_ROW_SLOPE = 8.0
# 桥前半宽：以用户最后提供的 163547 截图红线（桥左外沿）为中线参考。
# 同一行 half = 红线x - 左泡沫内沿x；不是到图像中轴的距离，也不自动清零误差。
# 截图几何近似标定，直接支持 y≈212~370（640x480）；范围外为透视外推。
# half/W = slope*(y/H) + intercept；y=300/360/432 时约 254/347/458px。
# 仅桥前默认先验；已有双边实测模型仍优先，桥后急弯和桥上路径不变。
PRE_BRIDGE_HALF_WIDTH_SLOPE = 1.15716
PRE_BRIDGE_HALF_WIDTH_INTERCEPT = -0.32589

# ---------- 巡线中点的静态偏置（像素）----------
# 车明明摆在车道正中，屏幕上 err 却不是 0，就用这个把中性点扳回来：
#     TARGET_OFFSET_PIXELS = 摆正时的 err × 320
#   例：摆正时 err=+0.13 → 填 +42；err=-0.08 → 填 -26
# 正值 = 把跟踪中心往右挪（err 变负）→ 车会往左修正；负值相反。
# 标定方法：车摆直道正中、enable_motion:=false，看屏幕 err，代入上式。
TARGET_OFFSET_PIXELS = 0.0

# ---------- 黄色赛道边缘 HSV（仅巡线，不改变场景识别） ----------
# OpenCV 8 位 HSV：H=0~179，S/V=0~255。
# 这里是节点默认值；config/shiwai.yaml 的 lane_yellow_* 参数优先。
LANE_YELLOW_H_MIN = 24
LANE_YELLOW_H_MAX = 40
LANE_YELLOW_S_MIN = 35
LANE_YELLOW_V_MIN = 70

# ---------- 巡线掩膜中的黄黑条纹排除（不影响场景识别） ----------
# 不再把"靠近暗地面"的黄边当作警示胶带；必须有局部重复的黄/黑交替。
STRIPE_MIN_RUN_PIXELS = 2            # 单段颜色至少连续 2 像素，忽略单像素噪声。
STRIPE_MAX_UNKNOWN_GAP_PIXELS = 4    # 只允许短小的颜色过渡；不跨越大段灰/白地面。
STRIPE_MIN_ALTERNATING_RUNS = 4      # 黄-黑-黄-黑，或黑-黄-黑-黄（至少 3 次交替）。
STRIPE_MAX_SPAN_RATIO = 0.16         # 交替序列总长度上限，相对当前搜索图像的宽度。
STRIPE_SUPPORT_PAD_PIXELS = 5        # 仅扩展已确认条纹的支持域，不扩展所有黑色。
STRIPE_COMPONENT_MIN_OVERLAP = 0.15  # 黄色连通块至少 15% 受到条纹支持才删除。

# ---------- 速度 ----------
# 直线巡线目标速度（m/s）。整体加快或减慢主要改这个。
FORWARD_SPEED = 0.30   #0.3

# 转弯自动降速后的最低前进速度（m/s）。
MINIMUM_FORWARD_SPEED = 0.2  #0.2

# 最大转向角速度（rad/s）。弯道转不过去就略增大。
# 0.38 → 0.48（提速一档）：170334 实测弯中 74 帧顶在 0.38 上，它是转弯耗时的
# 唯一瓶颈（t = 转角/角速度）。0.48 大约让大弯从 7.55s 缩到 6.0s。
# 风险：泡沫出境更快，若出现 TURN LOST 就先退回 0.43 或把
# TURN_HEADING_RAMP_FRAMES 从 8 加到 10。回退就是改回 0.38。
MAX_ANGULAR_SPEED = 0.48  #0.4，0.55，0.45，0.4，0.38 -> 0.48

# 转向比例增益：err=1 时输出的角速度 = STEERING_KP × MAX_ANGULAR_SPEED。
# 偏大 → 左右画龙（实测峰峰 40cm 以上），冲桥会被甩下去；
# 偏小 → 回正慢、进弯切内圈。画龙降 0.7，抢弯提 0.9。
STEERING_KP = 0.72  #0.75

# 转弯减速比例，范围 0~1。越大，转弯时越慢。
CURVE_SLOWDOWN = 0.40 #0.6

# 看见单边桥黄黑胶带后冲上桥面的速度（m/s），应比 FORWARD_SPEED 大。
BRIDGE_CLIMB_SPEED = 0.5  #0.4，0.45

# 视觉预告 + IMU 上桥/回平确认。所有新参数只在 lin 文件内维护。
BRIDGE_IMU_LEGACY_ORDER = True  # 当前桥接 x,y,z,w 中实际存的是 q0,q1,q2,q3。
BRIDGE_ACCELERATION = 0.60     # 桥区加速上限 m/s²；平地保持原 0.25。
BRIDGE_BASE_SECONDS = 0.60
BRIDGE_BASE_RATE_DEG = 8.0
BRIDGE_SENSOR_TIMEOUT = 0.50
BRIDGE_SENSOR_START_GRACE = 2.0
BRIDGE_ENTER_ROLL_DEG = 4.0    # 相对进入桥区前基准；不用偏航或单次冲击触发。
BRIDGE_ENTER_CONFIRM_SECONDS = 0.15
# 离桥采用单向低速确认，避免 exit_wait/crossing 反复切换继续冲桥。
# 这些角度均相对桥前基准；速度/距离需要实车低速验证，轮式里程不是地面真值。
BRIDGE_EXIT_START_ROLL_DEG = 3.0
BRIDGE_EXIT_RETURN_RATIO = 0.55  # 至少从峰值回落约一半才开始减速。
BRIDGE_EXIT_ROLL_DEG = 3.0
BRIDGE_EXIT_PITCH_DEG = 5.0
BRIDGE_RETURN_DELTA_DEG = 2.0
BRIDGE_EXIT_ROLL_RATE_DEG = 15.0
BRIDGE_EXIT_PITCH_RATE_DEG = 20.0
BRIDGE_EXIT_STABLE_SECONDS = 0.30  # 有效新 IMU 样本的累计平稳时间。
BRIDGE_EXIT_SPEED = 0.16          # m/s，离桥确认不再请求冲桥速度。
# exit_slow 的前进预算。预算用完进 exit_hold，以前那里是直接发 0。
# 1.20 → 2.00 / 0.25 → 0.50：给够时间让它在滚动中确认回平、直接进 done，
# 尽量连 exit_hold 都不进。
BRIDGE_EXIT_TIMEOUT = 2.00        # 前进确认预算（秒）；用完转低速滚行确认。
BRIDGE_EXIT_MAX_DISTANCE = 0.50   # 轮式里程仅限制继续直冲，不单独判定故障。
# exit_hold 不再停车：保持这个速度滚行，一边滚一边回平确认。
# 211948 实测原来那里直接发 0，4.533s→4.600s 一帧从 0.24 砸到 0.00
# （等效 −3.6 m/s²），接着又从 0 以 0.25 m/s² 爬回 0.16 —— 这就是下桥顿挫。
BRIDGE_EXIT_HOLD_SPEED = 0.16
BRIDGE_EXIT_SETTLE_TIMEOUT = 3.0  # 滚行确认一直不回平才报故障；不强判离桥。
BRIDGE_EXIT_RETILT_ROLL_DEG = 6.0
BRIDGE_EXIT_RETILT_PITCH_DEG = 12.0
BRIDGE_EXIT_RETILT_SECONDS = 0.15
BRIDGE_MIN_CROSS_SECONDS = 0.80
BRIDGE_MIN_DISTANCE = 0.35     # 仅附加约束：轮式里程会受打滑影响。
BRIDGE_EXIT_CLEARANCE = 0.04   # 回平后的小段离桥余量；仍须满足总跨桥距离。
# 过桥后额外要求车身真的稳下来，才把控制权交回桥后 WAIT。
# BridgeTraversal 的 done 只要求 0.30 s 累计平稳 + 4 cm 余量，实测下桥后仍有
# 横滚余摆；带着摆动进 WAIT 会把参考航向记歪，直行段一开始就是斜的。
BRIDGE_SETTLE_SECONDS = 0.0      # 连续平稳这么久才放行0.8
BRIDGE_SETTLE_ROLL_DEG = 2.0
BRIDGE_SETTLE_PITCH_DEG = 3.0
BRIDGE_SETTLE_RATE_DEG = 10.0
BRIDGE_SETTLE_TIMEOUT = 2.50      # 长时间稳不下来也要放行，不能一直卡在过桥后
BRIDGE_SETTLE_SPEED = 0.10        # 稳定确认期间的低速直行速度
BRIDGE_PROGRESS_SPEED = 0.04
BRIDGE_APPROACH_TIMEOUT = 15.0
BRIDGE_CROSS_TIMEOUT = 12.0    # 超时是异常停车，绝不设置“已过桥”。
BRIDGE_STALL_TIMEOUT = 2.0
BRIDGE_LOST_LINE_HOLD = 0.80   # 短时丢线继续保航向，随后降低速度。
BRIDGE_LOST_LINE_SPEED = 0.18
BRIDGE_LOST_LINE_TIMEOUT = 2.50
BRIDGE_HEADING_KP = 1.0
BRIDGE_MAX_W = 0.20
BRIDGE_VISUAL_MAX_W = 0.10
# 桥上单边视觉只复用本帧检测结果，不重复处理图像，也不设单边持续时间上限。
BRIDGE_SINGLE_MIN_CONFIDENCE = 0.85
BRIDGE_SINGLE_MIN_ROWS = 40
BRIDGE_SINGLE_MAX_OTHER_RATIO = 0.25  # 少量另一侧点不能冒充可靠双边。
BRIDGE_SINGLE_IMU_RATIO = 0.50       # IMU 辅助不超过单边视觉修正幅度的一半。
# 保留旧参数加载兼容；新状态机不使用这些秒数判断离桥。
BRIDGE_CLIMB_SECONDS = 2.0
BRIDGE_MIN_HOLD_SECONDS = 2.5
BRIDGE_PASS_SECONDS = 2.0  #2.2

# 黄黑胶带进入画面这个高度范围，才算“靠近桥面”（0=画面顶，1=画面底）。
# 框顶太靠上会过早开始计时，走到桥头加速/右偏已经结束。
BRIDGE_NEAR_TOP_RATIO = 0.52
BRIDGE_NEAR_BOTTOM_RATIO = 0.92

# 冲桥加速结束后、右偏还没取消时的速度比例（相对 FORWARD_SPEED）。
# 防止 6 秒加速用完人还在桥上，掉回 0.20 又没动力。
BRIDGE_ON_RAMP_SPEED_SCALE = 1.50

# 已经取消右偏、到达人行道之前的速度比例。
BRIDGE_SPEED_SCALE = 1.00

# 隧道下蹲通过时的速度比例。
# 2.0 → 0.60 m/s 太狠：185159 实测 27.7~28.3s 一直是 v=0.57~0.59，
# 终点三个大字就是在这段被甩过去的。而且 0.60 m/s 在 max_linear_acceleration
# =0.25 m/s² 下刷到停要 2.36s、滑行 0.70m，而 FINISH_DRIVE_IN_SECONDS=2.0
# @0.14 只有 0.28m 余量 —— 就算立刻认出终点也停不住。
# 降到 1.2(0.36 m/s)：刷车距离 0.26m，刚好在余量内。
TUNNEL_SPEED_SCALE = 1.8 #1.2

# ---------- 人行道 ----------
# 停稳后再计时的秒数。规则要求停车 5 秒。
SIDEWALK_STOP_SECONDS = 3.0 #5

# 斑马线搜索框（0=画面顶远处，1=画面底近处）。
# TOP 越大，越要斑马线靠近车才停车。太小会在进入停车区之前就停。
CROSSWALK_TOP_RATIO = 0.72
CROSSWALK_BOTTOM_RATIO = 0.94
CROSSWALK_CONFIRM_FRAMES = 6.0
# 5 → 4：164509 最后一次只数到 4 条（条数够停车了，是门槛把 4 拒掉了）。
# yaml 里写的也是 min_crosswalk_stripes: 4，只是被 _apply_field_tuning 覆盖成 5。
MIN_CROSSWALK_STRIPES = 4
# 框内允许的黄像素占比。旧值 0.16 太严：164509 第 315 帧斑马线 6 条完整在框里，
# 但左侧那大块黄泡沫（斜向白带）占了框的 ~24%，`yellow_frac > 0.16` 直接 early
# return，HUD 就显示 CW 0，一整个 16~25s 都没停车。0.35 仍然能把“整框黄”拒掉。
CROSSWALK_MAX_YELLOW_FRAC = 0.35

# ---------- 腿高 / 隧道下蹲 ----------
# 正常站立腿高（米）。底盘默认约 0.25，允许范围大约 0.14~0.36。
STAND_HEIGHT = 0.20 #0.25

# 过隧道时的下蹲腿高（米）。越小蹲得越明显，不要低于 0.14。
CROUCH_HEIGHT = 0.16

# 升降腿速度（m/s）。太大容易前后晃。
HEIGHT_RATE = 0.20

# 从开始下蹲算起，至少蹲这么多秒才允许起身。
MIN_CROUCH_SECONDS = 4.0

# 隧道入口黄黑胶带离开画面后，再继续蹲行这么多秒，避免洞里起身。
TUNNEL_PASS_SECONDS =0.1

# 最长下蹲时间（秒）。到点强制起身，防止一直蹲着。
MAX_CROUCH_SECONDS = 14.0

# 出隧道后丢线宽限（秒）。洞口反光容易丢黄线，3 秒后再停车。
POST_TUNNEL_LOST_GRACE_SECONDS = 1.5

# ---------- 终点 ----------
# 终点是地上三个大黑字，不是斑马线。过完隧道后认到就停车。
# TOP 越大越靠近车才停。太小会在终点前就停，太大可能压过终点才停。
FINISH_TOP_RATIO = 0.80
FINISH_BOTTOM_RATIO = 0.96
# 6 → 3 → 2。204944 实测：真终点字（`FN 3 HIT`）只活了 410、411 两帧（0.13s），
# TUNNEL_SPEED_SCALE=2.0 → v=0.59，字在 FINISH 框里一闪就过。3 帧永远差一帧。
# _detect_finish 自己要求 2~6 个黑块、跨度>=0.38W、平边<=2.4、近端>=8%、
# 黑像素 0.07~0.62，加上下面 2.5s 时间门挡住隧道口胶带，2 帧够用。
FINISH_CONFIRM_FRAMES = 2.0
# 黑字必须落到搜索框下部（靠近车）才算到达，只看见远处大字不算。
FINISH_NEAR_RATIO = 0.40
# 出隧道后再走这么多秒才允许认终点，避免把隧道口虚线/远处大字当成终点。
# 8.0 太晚：185159 里 _passed_tunnel 到 30.6s 才置 Y，闸门要到 38.6s 才开，
# 而终点字 28.0~28.8s 就已经过去了。降到 2.0，配合下面隧道内就开始认。
FINISH_AFTER_TUNNEL_SECONDS = 2.0
# 隧道内下蹲超过这么多秒后就开始允许认终点：终点字可能就在隧道末段/出口，
# 等 _passed_tunnel 置位已经太晚（185159 实测差 2.6s）。
# 4.0 → 2.0：192546 下蹲起于 24.2s，终点字 28.9s 才出现，2.0 已经够用。
# 2.0 → 5.0（于萧阳 10.29）：实测车从斑马线停稳起步，2 秒只走约 0.3m，
# 斑马线/隧道口胶带的黑块还在画面底部，满足 _detect_finish → 连续 3 帧就误判终点。
# 5.0 → 2.5（204944 实测）：真终点字出现在下蹲后 3.5s，5.0 的闸门比它晚 1.5s，
# 字一帧都录不到确认就过了；而隧道口胶带那波误报在下蹲后 0.8~1.4s，
# 2.5 正好卡在两者中间：胶带进不来，真字来得及。
FINISH_IN_TUNNEL_MIN_SECONDS = 3.5  #2.0->5.0->2.5
# 认出终点后再往前拱这么久再停，把车身开进终点带。太短会停在字前，太长会冲出垫子。
FINISH_DRIVE_IN_SECONDS = 1.2 #2.0
# ⚠ 这个值不是"低速"，是"拱入速度"。原来 0.14 比隧道巡航(0.33~0.36)低太多，
# 加上 _rate_limit 里减速是 3 倍率（lin_step *= 3.0），实测 184532 录像：
#   f454 v=0.33 → f456 v=0.29 → f458 v=0.18 → f460 v=0.14（0.27s 内掉完）
# 之后 1.9s 一直只有 0.14，全程只走了约 0.34m，看起来就是"认到终点立刻停住"。
# 而且同一帧 h 从 0.16 抬到 0.20，起身 + 掉速叠在一起更像急刹。
# 抬到 0.30 后：0.33→0.30 一帧完成，2s × 0.30 = 0.60m，才是真往前走 2 秒。
FINISH_DRIVE_IN_SPEED = 0.30  #0.14

# ---------- 识别画面录像 ----------
# 默认把带框的调试画面录下来，跑完一圈后把程序停掉即可。
RECORD_DEBUG_VIDEO = True
RECORDINGS_DIR = str(Path.home() / "wheel_robot" / "recordings")
# HDMI 本机屏：True=调试窗全屏。录像仍是原始 640x480，不受影响。
DEBUG_FULLSCREEN = True


@dataclass
class DetectionResult:
    valid: bool
    error: float
    confidence: float
    reason: str
    debug_frame: np.ndarray
    lane_width_pixels: float = 0.0
    left_line_rows: int = 0
    right_line_rows: int = 0
    left_edge_x: float = 0.0
    curve_mode: bool = False
    lateral_error: float = 0.0
    heading_error: float = 0.0
    turn_strength: float = 0.0
    measured_points: int = 0
    boundary_span_pixels: float = 0.0
    boundary_side: str = ""
    band_weights: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    fill_weight_fraction: float = 0.0
    logic_rejected_points: int = 0
    turn_prior_applied: bool = False
    width_source: str = "prior"
    turn_exit_ready: bool = False
    heading_slope: float = 0.0
    turn_entry_ready: bool = False
    turn_entry_row: float = 0.0


@dataclass
class SceneObservation:
    crosswalk: bool
    hazard: bool
    hazard_near: bool
    finish: bool
    crosswalk_score: float
    hazard_score: float
    finish_score: float
    stripe_count: int
    finish_blobs: int


class DualYellowLineDetector:
    """在画面下部寻找成对的黄色泡沫边界，并跟随其中点。"""

    def __init__(
        self,
        search_top_ratio: float,
        search_bottom_ratio: float,
        search_left_ratio: float,
        search_right_ratio: float,
        yellow_h_min: int,
        yellow_h_max: int,
        yellow_s_min: int,
        yellow_v_min: int,
        black_value_max: int,
        black_saturation_max: int,
        min_boundary_lane_contrast: float,
        morphology_kernel: int,
        min_line_width_ratio: float,
        max_line_width_ratio: float,
        expected_line_width_ratio: float,
        width_tolerance_ratio: float,
        min_lane_width_ratio: float,
        max_lane_width_ratio: float,
        expected_lane_width_ratio: float,
        lane_width_tolerance_ratio: float,
        max_center_jump_ratio: float,
        min_valid_row_fraction: float,
        min_confidence: float,
        target_offset_pixels: float,
    ) -> None:
        self.search_top_ratio = search_top_ratio
        self.search_bottom_ratio = search_bottom_ratio
        self.search_left_ratio = search_left_ratio
        self.search_right_ratio = search_right_ratio
        self.yellow_h_min = yellow_h_min
        self.yellow_h_max = yellow_h_max
        self.yellow_s_min = yellow_s_min
        self.yellow_v_min = yellow_v_min
        self.black_value_max = black_value_max
        self.black_saturation_max = black_saturation_max
        self.min_boundary_lane_contrast = min_boundary_lane_contrast
        self.morphology_kernel = morphology_kernel
        self.min_line_width_ratio = min_line_width_ratio
        self.max_line_width_ratio = max_line_width_ratio
        self.expected_line_width_ratio = expected_line_width_ratio
        self.width_tolerance_ratio = width_tolerance_ratio
        self.min_lane_width_ratio = min_lane_width_ratio
        self.max_lane_width_ratio = max_lane_width_ratio
        self.expected_lane_width_ratio = expected_lane_width_ratio
        self.lane_width_tolerance_ratio = lane_width_tolerance_ratio
        self.max_center_jump_ratio = max_center_jump_ratio
        self.min_valid_row_fraction = min_valid_row_fraction
        self.min_confidence = min_confidence
        self.target_offset_pixels = target_offset_pixels
        self.single_left_half_ratio = SINGLE_LEFT_HALF_RATIO
        self.single_right_half_ratio = SINGLE_RIGHT_HALF_RATIO
        self.near_lock_ratio = NEAR_LOCK_RATIO
        self.min_single_coverage = 0.22
        self.base_min_valid_row_fraction = min_valid_row_fraction
        self.base_min_single_coverage = 0.22
        self.base_near_lock_ratio = NEAR_LOCK_RATIO
        self.force_left_line = False
        # 连续内沿模式：由节点切换，桥上仍用原检测路径。
        self.column_scan = False
        self.expected_turn_direction = 0  # 0=通用；+1=桥后到人行道前的唯一右弯。
        self.prefer_left_line = True
        self.pre_bridge_guard = False
        # >0 表示 WAIT 还没跑够时间，剩余秒数；此时不允许累计入弯计数。
        self.turn_entry_hold = 0.0
        # 斑马线被拒的原因，只写 HUD。
        self.cw_reason = ''
        self.width_profile = "standing"
        self._curve_tracks = {}
        self._curve_track_key = None
        self._curve_control_side = None
        self._curve_left_return_frames = 0
        self._curve_width_model = None
        self._curve_width_key = None
        self._curve_turn_entry_ready = False
        self._curve_turn_entry_count = 0
        self._curve_turn_entry_frames = 0
        self._curve_exit_armed = False
        self._curve_exit_ready = False
        self._curve_exit_count = 0
        self._curve_exit_frames = 0
        self._turn_ff_strength = 0.0
        self._hud_cx: Optional[float] = None
        self._hud_cy: Optional[float] = None

    @staticmethod
    def _runs(row: np.ndarray) -> List[Tuple[int, int]]:
        # 等价于两端补零；避免每一行调用通用 np.pad 的参数解析。
        padded = np.empty(row.size + 2, dtype=np.int8)
        padded[0] = padded[-1] = 0
        padded[1:-1] = row > 0
        changes = np.diff(padded)
        return list(
            zip(
                np.flatnonzero(changes == 1).tolist(),
                np.flatnonzero(changes == -1).tolist(),
            )
        )

    @staticmethod
    def _smooth_xs(xs: List[float], window: int = 7) -> List[float]:
        if len(xs) < 3:
            return xs
        radius = max(1, window // 2)
        smoothed: List[float] = []
        for index in range(len(xs)):
            lo = max(0, index - radius)
            hi = min(len(xs), index + radius + 1)
            smoothed.append(float(np.mean(xs[lo:hi])))
        return smoothed

    @staticmethod
    def _mask_inner_points(
        mask: np.ndarray, x0: int, y0: int, inner: str, step: int = 5
    ) -> List[Tuple[int, int]]:
        points: List[Tuple[int, int]] = []
        height = mask.shape[0]
        for row_y in range(height - 1, -1, -step):
            cols = np.flatnonzero(mask[row_y] > 0)
            if cols.size == 0:
                continue
            col = int(cols[-1] if inner == "right" else cols[0])
            points.append((x0 + col, y0 + row_y))
        return points

    def _draw_path(
        self,
        debug: np.ndarray,
        points: List[Tuple[float, int]],
        y0: int,
        color: Tuple[int, int, int],
        thickness: int = 2,
    ) -> None:
        if len(points) < 2:
            return
        ordered = sorted(points, key=lambda item: item[1])
        step = max(1, len(ordered) // 28)
        sampled = ordered[::step]
        if sampled[-1] != ordered[-1]:
            sampled.append(ordered[-1])
        xs = self._smooth_xs([item[0] for item in sampled], 5)
        polyline = np.array(
            [(int(x), y0 + int(y)) for x, (_, y) in zip(xs, sampled)],
            dtype=np.int32,
        )
        cv2.polylines(debug, [polyline], False, color, thickness, cv2.LINE_AA)

    @staticmethod
    def _stripe_exclusion_mask(
        yellow: np.ndarray, black: np.ndarray, local_only: bool = False
    ) -> np.ndarray:
        """只排除有局部重复黄黑交替支持的黄色连通块。

        一个连续黄边旁边只有一片暗地面时，不满足交替条件，应当保留。
        该掩膜只供巡线使用；不调用或修改 TrackSceneDetector 的判断。
        """
        excluded = np.zeros_like(yellow)
        if yellow.size == 0 or not np.any(yellow) or not np.any(black):
            return excluded

        # 黄色优先，确保同一像素不会同时被编码成黄和黑。
        state = np.where(yellow > 0, 1, np.where(black > 0, 2, 0)).astype(np.uint8)
        evidence = np.zeros_like(yellow)
        min_run = STRIPE_MIN_RUN_PIXELS
        max_gap = STRIPE_MAX_UNKNOWN_GAP_PIXELS
        needed = STRIPE_MIN_ALTERNATING_RUNS
        max_span = max(16, int(yellow.shape[1] * STRIPE_MAX_SPAN_RATIO))

        def scan(scanlines: np.ndarray, support: np.ndarray) -> None:
            # 一次批量提取全部扫描线的颜色游程；后续的间隔、交替次数、
            # 支持跨度判断保持原样。仍扫描每一行、每一列，不降采样。
            line_count, line_size = scanlines.shape
            if line_size == 0:
                return
            eligible = (
                (np.count_nonzero(scanlines == 1, axis=1) >= 2 * min_run)
                & (np.count_nonzero(scanlines == 2, axis=1) >= 2 * min_run)
            )
            edges = np.empty((line_count, line_size + 1), dtype=bool)
            edges[:, 0] = edges[:, -1] = True
            edges[:, 1:-1] = scanlines[:, 1:] != scanlines[:, :-1]
            edge_rows, edge_cols = np.nonzero(edges)
            same_row = edge_rows[:-1] == edge_rows[1:]
            rows = edge_rows[:-1][same_row]
            starts = edge_cols[:-1][same_row]
            ends = edge_cols[1:][same_row]
            colors = scanlines[rows, starts]
            keep = eligible[rows] & (colors != 0) & (ends - starts >= min_run)
            rows, starts, ends, colors = (
                array[keep].tolist() for array in (rows, starts, ends, colors)
            )
            previous_line = -1
            runs: List[Tuple[int, int, int]] = []
            for line_index, start, end, color in zip(rows, starts, ends, colors):
                if line_index != previous_line:
                    runs = []
                    previous_line = line_index
                if runs and start - runs[-1][1] <= max_gap:
                    if color == runs[-1][2]:
                        runs[-1] = (runs[-1][0], end, color)
                    else:
                        runs.append((start, end, color))
                else:
                    runs = [(start, end, color)]
                if len(runs) >= needed:
                    group = runs[-needed:]
                    if group[-1][1] - group[0][0] <= max_span:
                        support[line_index, group[0][0]:group[-1][1]] = 255

        # 两个方向都扫描，因此不要求胶带一定水平或垂直。
        scan(state, evidence)
        scan(state.T, evidence.T)
        # 去掉孤立单行/单列的交替证据，要求相邻扫描线也提供支持。
        evidence = cv2.morphologyEx(
            evidence, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
        )
        if not np.any(evidence):
            return excluded
        pad_size = 2 * STRIPE_SUPPORT_PAD_PIXELS + 1
        supported = cv2.dilate(
            evidence, np.ones((pad_size, pad_size), dtype=np.uint8)
        )

        if local_only:
            return cv2.bitwise_and(yellow, supported)

        # 按黄色连通块删除，不扩大暗地面的边缘。支持比例门限可避免仅
        # 擦到条纹支持域的一小部分，就把整条长黄色海绵一起删除。
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            yellow, connectivity=8
        )
        overlap = np.bincount(labels[supported > 0], minlength=count)
        required = np.maximum(
            2.0, STRIPE_COMPONENT_MIN_OVERLAP * stats[:, cv2.CC_STAT_AREA]
        )
        remove = overlap >= required
        remove[0] = False  # 背景不是黄色连通块。
        excluded[remove[labels]] = 255
        return excluded

    def _yellow_mask(self, hsv: np.ndarray, local_only: bool = False) -> np.ndarray:
        yellow = cv2.inRange(
            hsv,
            np.array(
                (self.yellow_h_min, self.yellow_s_min, self.yellow_v_min),
                dtype=np.uint8,
            ),
            np.array((self.yellow_h_max, 255, 255), dtype=np.uint8),
        )
        black = cv2.inRange(
            hsv,
            np.array((0, 0, 0), dtype=np.uint8),
            np.array(
                (180, self.black_saturation_max, self.black_value_max),
                dtype=np.uint8,
            ),
        )
        striped = self._stripe_exclusion_mask(yellow, black, local_only)
        return cv2.bitwise_and(yellow, cv2.bitwise_not(striped))

    # ------------------------------------------------------------------
    # 最长白列 + 逐行边界搜索 + 远端拟合延伸
    # 参考智能车常用的 Longest_White_Column_Edge_Detection：
    #   1) 统计每列从画面底部向上的连续白点数
    #   2) 最左 / 最右的「最长白列」分别作为左线、右线的种子列
    #   3) 自底向上，从种子列向外找「白→黑」跳变得到左右边界
    #   4) 本条补充：丢线的行用已检出行的一阶拟合外推（远端 → 近端）
    # ------------------------------------------------------------------
    @staticmethod
    def _column_run_lengths(mask: np.ndarray) -> np.ndarray:
        """每列从最底行开始向上的连续白点数。"""
        white = mask > 0
        height, width = white.shape
        lengths = np.zeros(width, dtype=np.int32)
        if height == 0:
            return lengths
        alive = white[height - 1].copy()
        lengths += alive
        for row_y in range(height - 2, -1, -1):
            alive &= white[row_y]
            if not alive.any():
                break
            lengths += alive
        return lengths

    @staticmethod
    def _fit_extend(
        edges: np.ndarray, width: int, max_rows: int, min_points: int
    ) -> Tuple[np.ndarray, np.ndarray, bool]:
        """丢失行用已检出行的一阶拟合外推。返回 (补齐后, 外推行掩码, 是否拟合)。"""
        out = edges.copy()
        filled = np.zeros(edges.shape, dtype=bool)
        index = np.flatnonzero(np.isfinite(edges))
        if index.size < min_points:
            return out, filled, False
        missing = np.flatnonzero(~np.isfinite(edges))
        if missing.size == 0:
            return out, filled, False
        gap = np.min(np.abs(missing[:, None] - index[None, :]), axis=1)
        near = gap <= max_rows
        if not np.any(near):
            return out, filled, False
        slope = float(np.polyfit(index.astype(float), edges[index], 1)[0])
        slope = float(np.clip(slope, -COLUMN_FIT_MAX_SLOPE, COLUMN_FIT_MAX_SLOPE))
        intercept = float(np.mean(edges[index]) - slope * float(np.mean(index)))
        rows = missing[near]
        out[rows] = np.clip(slope * rows + intercept, 0.0, float(width - 1))
        filled[rows] = True
        return out, filled, True

    def _column_scan_edges(
        self, mask: np.ndarray, target_x: float
    ) -> Tuple[np.ndarray, np.ndarray, int, int, int]:
        """返回 (left_inner, right_inner, left_col, right_col, stop_rows)。

        数组长度 = ROI 行数，索引 0 = ROI 顶部、末位 = ROI 底部（贴着车）。
        left_inner 是左泡沫的右内沿，right_inner 是右泡沫的左内沿。
        """
        height, width = mask.shape
        left_inner = np.full(height, np.nan)
        right_inner = np.full(height, np.nan)
        if height < 4 or width < 4 or not np.any(mask):
            return left_inner, right_inner, -1, -1, 0

        lengths = self._column_run_lengths(mask)
        counts = np.count_nonzero(mask > 0, axis=0)
        if int(counts.max()) <= 0:
            return left_inner, right_inner, -1, -1, 0
        # 种子列：左右半区各取「白点总数最多」的一列。
        # 不能照搬「从底行往上连续白点数最多」：赛道上左侧近处泡沫贴着画面左下角，
        # 右侧远处泡沫悬在画面上半部，它所在列的底部连续长度恒为 0，
        # 于是两个种子会同时落进左边那根泡沫，右边界永远找不到。
        split = int(np.clip(round(target_x), 1, width - 2))
        left_half = counts[:split]
        right_half = counts[split:]
        # 某一半区完全没有白点就给 -1，表示该侧这条线不存在，
        # 不能把 argmax 的 0 当成真的种子列。
        left_col = int(np.argmax(left_half)) if int(left_half.max()) > 0 else -1
        right_col = (
            split + int(np.argmax(right_half)) if int(right_half.max()) > 0 else -1
        )
        if left_col < 0 and right_col < 0:
            return left_inner, right_inner, -1, -1, 0
        stop_rows = (
            int(min(lengths[left_col], lengths[right_col]))
            if left_col >= 0 and right_col >= 0
            else 0
        )
        search_start = 0

        min_line_width = max(2, int(width * self.min_line_width_ratio))
        max_line_width = max(
            min_line_width + 1, int(width * self.max_line_width_ratio)
        )
        padded = np.zeros((height, width + 2), dtype=np.int8)
        padded[:, 1:-1] = mask > 0
        changes = np.diff(padded, axis=1)
        run_rows, run_starts = np.nonzero(changes == 1)
        _, run_ends = np.nonzero(changes == -1)
        run_widths = run_ends - run_starts
        keep = (run_widths >= min_line_width) & (run_widths <= max_line_width)
        row_runs: List[List[Tuple[int, int]]] = [[] for _ in range(height)]
        for row_y, start, end in zip(
            run_rows[keep].tolist(),
            run_starts[keep].tolist(),
            run_ends[keep].tolist(),
        ):
            row_runs[row_y].append((start, end))

        for row_y in range(height - 1, search_start - 1, -1):
            runs = row_runs[row_y]
            if not runs:
                continue
            left_run = next(
                (item for item in runs if item[0] <= left_col < item[1]), None
            )
            right_run = next(
                (item for item in runs if item[0] <= right_col < item[1]), None
            )
            # 种子列在这一行不是白的 → 该侧这条线在本行不可见，留空交给
            # _fit_extend 外推。绝不能退化成"最近的 run"：近处泡沫占满左下半屏时
            # 那样会把远处碎块当右边界，甚至把同一条泡沫当成左右两条线。
            if left_run is None and right_run is None:
                continue
            if left_run is not None and left_run == right_run:
                # 两个种子落在同一条泡沫里，只能当单边线用
                run_center = 0.5 * (left_run[0] + left_run[1] - 1)
                if run_center <= target_x:
                    left_inner[row_y] = float(left_run[1] - 1)
                else:
                    right_inner[row_y] = float(left_run[0])
                continue
            if left_run is not None:
                left_inner[row_y] = float(left_run[1] - 1)
            if right_run is not None:
                right_inner[row_y] = float(right_run[0])
        return left_inner, right_inner, left_col, right_col, stop_rows

    def _centers_from_edges(
        self,
        left_inner: np.ndarray,
        right_inner: np.ndarray,
        width: int,
        target_x: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """补全丢失行 → 逐行算虚拟中点。"""
        left_fill, left_ext, _ = self._fit_extend(
            left_inner, width, COLUMN_FIT_MAX_ROWS, COLUMN_FIT_MIN_POINTS
        )
        right_fill, right_ext, _ = self._fit_extend(
            right_inner, width, COLUMN_FIT_MAX_ROWS, COLUMN_FIT_MIN_POINTS
        )
        min_lane = width * self.min_lane_width_ratio
        max_lane = width * self.max_lane_width_ratio
        centers = np.full(left_inner.shape, np.nan)
        for row_y in range(centers.size):
            left_x = left_fill[row_y]
            right_x = right_fill[row_y]
            if (
                np.isfinite(left_x)
                and np.isfinite(right_x)
                and min_lane <= (right_x - left_x) <= max_lane
            ):
                centers[row_y] = 0.5 * (left_x + right_x)
            elif np.isfinite(left_x):
                centers[row_y] = left_x + self.single_left_half_ratio * width
            elif np.isfinite(right_x):
                centers[row_y] = right_x - self.single_right_half_ratio * width
        # 单边半宽推出来的中点可能跑出画面，必须裁掉，否则调试绘制会越界。
        np.clip(centers, 0.0, float(width - 1), out=centers)
        return centers, left_fill, right_fill, left_ext, right_ext

    @staticmethod
    def _curve_fit_samples(points: np.ndarray, horizontal: bool, side: str,
                           width: int, height: int):
        """沿可观测方向拟合二次曲线；只补小缺口，不跨大段未知区域。"""
        axis = 0 if horizontal else 1
        u, v = points[:, axis], points[:, 1-axis]
        if len(u) < CURVE_MIN_POINTS or np.ptp(u) < CURVE_MIN_SPAN_RATIO * width:
            return None
        # 单点毛刺先按邻域中值剔除，再用整段曲线残差排除成簇的异常点。
        initial_points = len(points)
        original_points = points
        local = cv2.medianBlur(v.astype(np.float32).reshape(1, -1), 5).reshape(-1)
        coherent = np.abs(v-local) <= max(4.0, 0.01*width)
        points, u, v = points[coherent], u[coherent], v[coherent]
        if len(points) < CURVE_MIN_POINTS:
            return None
        origin, scale = float(np.mean(u)), max(1.0, float(np.ptp(u)))
        z = (u-origin)/scale
        keep = np.ones(len(u), dtype=bool)
        for _ in range(3):
            if np.count_nonzero(keep) < CURVE_MIN_POINTS:
                return None
            coef = np.polyfit(z[keep], v[keep], 2)
            residual = v - np.polyval(coef, z)
            median = float(np.median(residual[keep]))
            mad = float(np.median(np.abs(residual[keep]-median)))
            next_keep = np.abs(residual-median) <= max(3.0, 3.0*1.4826*mad)
            if np.array_equal(next_keep, keep):
                break  # 点集不再变化就停止拟合，避免占用下一帧的处理时间。
            keep = next_keep
        if np.mean(keep) < 0.65 or np.count_nonzero(keep) < CURVE_MIN_POINTS:
            return None
        coef = np.polyfit(z[keep], v[keep], 2)
        rms = float(np.sqrt(np.mean((v[keep]-np.polyval(coef, z[keep]))**2)))
        if rms > CURVE_MAX_FIT_RMS:
            return None
        measured = points[keep].copy()
        observed = u[keep].astype(int)
        # 外推长度同时受像素上限与实测跨度限制；远端最多约 10%。
        extension = min(CURVE_MAX_GAP_PIXELS, max(1, int(np.ptp(observed)*0.10)))
        limit = width if horizontal else height
        grid = np.arange(max(0, int(observed.min())-extension),
                         min(limit, int(observed.max())+extension+1))
        insertion = np.searchsorted(observed, grid)
        before = observed[np.clip(insertion-1, 0, len(observed)-1)]
        after = observed[np.clip(insertion, 0, len(observed)-1)]
        distance = np.minimum(np.abs(grid-before), np.abs(grid-after))
        small_gap = ((distance == 0) | ((after-before) <= CURVE_MAX_GAP_PIXELS)
                     | (grid < observed[0]) | (grid > observed[-1]))
        select = (distance <= extension) & small_gap
        grid, distance = grid[select], distance[select]
        fit = np.polyval(coef, (grid-origin)/scale)
        path = np.column_stack((grid, fit) if horizontal else (fit, grid))
        inside = (path[:, 0] >= 0) & (path[:, 0] < width) & (path[:, 1] >= 0) & (path[:, 1] < height)
        path, distance = path[inside], distance[inside]
        if len(path) < CURVE_MIN_POINTS:
            return None
        # 统一按近到远排列。横向边沿采用左右侧的前进方向，避免 y 接近水平时乱序。
        reverse = not horizontal
        if horizontal:
            endpoint_delta = float(np.median(path[-12:, 1])-np.median(path[:12, 1]))
            reverse = endpoint_delta > 0 if abs(endpoint_delta) > 8.0 else side == 'right'
        if reverse:
            path, distance = path[::-1].copy(), distance[::-1].copy()
        measured_support = distance == 0
        quality = np.where(measured_support, 1.0,
                           CURVE_FILL_WEIGHT*np.exp(-distance/max(1.0, extension)))
        return dict(path=path, measured=measured, support=measured_support,
                    quality=quality, side=side, horizontal=horizontal, rms=rms,
                    fit_fraction=float(np.mean(keep)),
                    rejected=np.vstack((original_points[~coherent], points[~keep])),
                    rejected_points=initial_points-int(np.count_nonzero(keep)))

    def _curve_match_side(self, points, horizontal, width):
        """只关联本帧实测点的身份；不把旧点混入本帧控制误差。"""
        axis = 0 if horizontal else 1
        matches = []
        near_now = points[points[:, 1] >= np.max(points[:, 1])-.10*np.ptp(points[:, 1]), 0]
        anchor = float(np.median(near_now))
        for side, track in self._curve_tracks.items():
            if track['horizontal'] != horizontal:
                continue
            old = track['measured']
            # 镜像边界可能在画面中部交叠，近端却位于相反两侧，不能认成同一条线。
            if abs(anchor-track['near_x']) > 2*LANE_SIDE_MATCH_RATIO*width:
                continue
            overlapping = points[(points[:, axis] >= old[0, axis])
                                 & (points[:, axis] <= old[-1, axis])]
            if len(overlapping) < CURVE_MIN_POINTS:
                continue
            predicted = np.interp(overlapping[:, axis], old[:, axis], old[:, 1-axis])
            residual = np.abs(overlapping[:, 1-axis]-predicted)
            distance = float(np.median(residual))
            if distance <= LANE_SIDE_MATCH_RATIO*width and np.percentile(residual, 80) <= 1.25*LANE_SIDE_MATCH_RATIO*width:
                matches.append((distance, side))
        matches.sort()
        if not matches or (len(matches) > 1 and matches[1][0]-matches[0][0] < 8.0):
            return None
        return matches[0][1]

    @staticmethod
    def _curve_edge_confidence(edge, width):
        span = float(np.linalg.norm(np.ptp(edge['measured'], axis=0)))
        return float((0.45*min(1.0, span/(0.30*width))
                      + 0.30*min(1.0, len(edge['measured'])/100.0)
                      + 0.25*max(0.0, 1.0-edge['rms']/CURVE_MAX_FIT_RMS))
                     * edge['logic_weight'])

    def _curve_fit_candidate(self, points, horizontal, side, width, height, y0, image_height):
        """桥前从近端沿连续边界拟合，截掉粘连远横边产生的持续突变。"""
        far_points = None
        if self.pre_bridge_guard and not horizontal and len(points) >= CURVE_MIN_POINTS:
            xs = cv2.medianBlur(points[:, 0].astype(np.float32).reshape(1, -1), 5).reshape(-1)
            dx, dy = np.diff(xs), np.diff(points[:, 1])
            jumps = np.flatnonzero(np.abs(dx) > PRE_BRIDGE_MAX_ROW_SLOPE*np.maximum(1.0, dy))
            # 短距离跳出又回到原线的是局部毛刺，交原有点级鲁棒拟合剔除，
            # 不因一个小凸块截掉其上方整段真实边线。
            paired = set()
            for a, b in zip(jumps[:-1], jumps[1:]):
                if (dx[a]*dx[b] < 0 and points[b+1, 1]-points[a, 1] <= CURVE_MAX_GAP_PIXELS
                        and abs(dx[a]+dx[b]) <= max(8.0, .25*max(abs(dx[a]), abs(dx[b])))):
                    paired.update((int(a), int(b)))
            persistent = [int(i) for i in jumps if int(i) not in paired]
            if persistent:
                cuts = [0]+[i+1 for i in persistent]+[len(points)]
                usable = [(start, end) for start, end in zip(cuts[:-1], cuts[1:])
                          if self._curve_has_near_support(
                              dict(horizontal=False, measured=points[start:end]), y0, image_height)]
                if not usable:
                    return None
                # 最近端若只是几个异常点，采用其前方仍有近端实测支持的连续段。
                # 不强行保留最底部碎片，也不让远处横边通过重新拟合混回来。
                start, end = max(usable, key=lambda part: points[part[1]-1, 1])
                far_points = np.vstack((points[:start], points[end:]))
                points = points[start:end]
        candidate = self._curve_fit_samples(points, horizontal, side, width, height)
        if candidate is not None and far_points is not None:
            candidate['rejected'] = np.vstack((candidate['rejected'], far_points))
            candidate['rejected_points'] += len(far_points)
        return candidate

    @staticmethod
    def _curve_has_near_support(candidate, y0: int, image_height: int) -> bool:
        """桥前只认可逐行实测内沿；外推点、横向像素数不提供近端支持。"""
        if candidate['horizontal']:
            return False
        # 非横向候选按 y 取点，每行至多一个点，拟合与片段合并均保持 y 升序。
        rows = candidate['measured'][:, 1]
        if len(rows) < max(CURVE_MIN_POINTS, int(round(PRE_BRIDGE_MIN_ROWS_RATIO*image_height))):
            return False
        if rows[-1]-rows[0] < PRE_BRIDGE_MIN_Y_SPAN_RATIO*image_height:
            return False
        near_row = PRE_BRIDGE_NEAR_ROW_RATIO*image_height-y0
        return np.count_nonzero(rows >= near_row) >= max(
            3, int(round(PRE_BRIDGE_MIN_NEAR_ROWS_RATIO*image_height)))

    def _curve_boundaries(self, mask: np.ndarray, x0: int, full_width: int,
                          y0: int, image_height: int):
        """一个连通泡沫只是一条边，不再因穿过图像中心而拆成左右两条。"""
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        height, width = mask.shape
        candidates = []
        order = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1][:6] + 1
        if self.pre_bridge_guard:
            # 先以连通块高度排除远处薄横条，再把可能到达近端的候选排在前面。
            # 在像素提取/拟合之前缩小候选集，避免大横条占满六个处理名额。
            ids = np.flatnonzero(
                (stats[1:, cv2.CC_STAT_AREA] >= 120)
                & (stats[1:, cv2.CC_STAT_HEIGHT]-1 >= PRE_BRIDGE_MIN_Y_SPAN_RATIO*image_height)
            )+1
            reaches_near = (stats[ids, cv2.CC_STAT_TOP]+stats[ids, cv2.CC_STAT_HEIGHT]-1+y0
                            >= PRE_BRIDGE_NEAR_ROW_RATIO*image_height)
            order = ids[np.lexsort((stats[ids, cv2.CC_STAT_AREA], reaches_near))[::-1][:6]]
        for label in order:
            bx, by, bw, bh, area = stats[label]
            if area < 120 or max(bw, bh) < CURVE_MIN_SPAN_RATIO*width:
                continue
            yy, xx = np.nonzero(labels[by:by+bh, bx:bx+bw] == label)
            xx, yy = xx+bx, yy+by
            near = xx[yy >= np.percentile(yy, 90)]
            near_x = float(np.median(near)) + x0
            side = 'left' if near_x < 0.5*full_width else 'right'
            # 桥前恢复同一图像行的中点，必须按行取内沿；桥后急弯保留按列拟合。
            horizontal = not self.pre_bridge_guard and bw > 1.8*bh
            if horizontal:
                edge = np.full(width, -1, dtype=int)
                np.maximum.at(edge, xx, yy)
                columns = np.flatnonzero((edge >= 0) & (edge < height-1))
                points = np.column_stack((columns, edge[columns])).astype(float)
            else:
                edge = np.full(height, -1 if side == 'left' else width, dtype=int)
                if side == 'left':
                    np.maximum.at(edge, yy, xx)
                else:
                    np.minimum.at(edge, yy, xx)
                rows = np.flatnonzero((edge > 0) & (edge < width-1))
                points = np.column_stack((edge[rows], rows)).astype(float)
            candidate = self._curve_fit_candidate(points, horizontal, side, width, height, y0, image_height)
            if candidate is not None:
                matched = self._curve_match_side(candidate['measured'], horizontal, full_width)
                if matched is not None and matched != side:
                    side = matched
                    if not horizontal:
                        edge = np.full(height, -1 if side == 'left' else width, dtype=int)
                        if side == 'left':
                            np.maximum.at(edge, yy, xx)
                        else:
                            np.minimum.at(edge, yy, xx)
                        rows = np.flatnonzero((edge > 0) & (edge < width-1))
                        points = np.column_stack((edge[rows], rows)).astype(float)
                    candidate = self._curve_fit_candidate(points, horizontal, side, width, height, y0, image_height)
                    if candidate is None:
                        continue
                candidate['area'] = int(area)
                candidate['near_x'] = near_x
                candidate['track_match'] = matched == side
                candidates.append(candidate)
        # 阴影/色差会把同一泡沫切成多个块。允许关联跨度有限、切线连续的观测，
        # 但大于 24px 的中间缺口仍不填点；“关联同一边”不等于“整段已看见”。
        changed = True
        while changed:
            changed = False
            for i in range(len(candidates)):
                for j in range(i+1, len(candidates)):
                    a, b = candidates[i], candidates[j]
                    if a['horizontal'] != b['horizontal']:
                        continue
                    if not a['horizontal'] and a['side'] != b['side']:
                        continue
                    axis = 0 if a['horizontal'] else 1
                    pa, pb = a['measured'], b['measured']
                    if pa[0, axis] > pb[0, axis]:
                        pa, pb = pb, pa
                    gap = pb[0, axis]-pa[-1, axis]
                    if not 0 < gap <= CURVE_ASSOCIATION_GAP_RATIO*width:
                        continue
                    sa = np.polyfit(pa[-16:, axis], pa[-16:, 1-axis], 1)[0]
                    sb = np.polyfit(pb[:16, axis], pb[:16, 1-axis], 1)[0]
                    jump = pb[0, 1-axis]-pa[-1, 1-axis]
                    if abs(sa-sb) > 0.75 or abs(jump-0.5*(sa+sb)*gap) > 8.0:
                        continue
                    points = np.vstack((pa, pb))
                    near_x = float(np.median(points[points[:, 1] >= np.percentile(points[:, 1], 90), 0]))+x0
                    matched = self._curve_match_side(points, a['horizontal'], full_width)
                    side = (matched
                            or (a['side'] if a['side'] == b['side'] else
                                ('left' if near_x < 0.5*full_width else 'right')))
                    joined = self._curve_fit_candidate(points, a['horizontal'], side, width, height, y0, image_height)
                    if joined is None or joined['fit_fraction'] < 0.85:
                        continue
                    joined.update(area=a['area']+b['area'], near_x=near_x,
                                  track_match=matched == side,
                                  rejected=np.vstack((joined['rejected'], a['rejected'], b['rejected'])),
                                  rejected_points=joined['rejected_points']+a['rejected_points']+b['rejected_points'])
                    candidates[i] = joined
                    candidates.pop(j)
                    changed = True
                    break
                if changed:
                    break
        if self.pre_bridge_guard:
            # 在片段关联后检查实测支持，允许真实近远片段关联，但不允许补点造证据。
            candidates = [c for c in candidates
                          if self._curve_has_near_support(c, y0, image_height)]
        # 赛道先验只降低明显反向的整条候选可信度。位置误差的正负不在此筛选，
        # 因此车身偏右时仍能向左纠偏；直线边沿也不会被当作反向急弯删除。
        for c in candidates:
            real_path = c['path'][c['support']]
            direction = real_path[-1]-real_path[0]
            angle = np.degrees(np.arctan2(abs(direction[0]), max(1.0, -direction[1])))
            vote = np.sign(direction[0])*np.clip((angle-CURVE_HEADING_START_DEG)/
                                               (CURVE_HEADING_FULL_DEG-CURVE_HEADING_START_DEG), 0, 1)
            contrary = -self.expected_turn_direction*vote
            c['logic_weight'] = float(1.0-0.75*np.clip((contrary-0.25)/0.5, 0, 1))
            c['confidence'] = self._curve_edge_confidence(c, full_width)
        # 每侧综合实测支持与方向一致性选择，防止色块/反向候选主导控制。
        edges = {side: max((c for c in candidates if c['side'] == side),
                           key=lambda c: (c['confidence'] >= .70, c['track_match'],
                                          len(c['measured'])*c['logic_weight']), default=None)
                 for side in ('left', 'right')}
        self._curve_tracks = {}
        for side, c in edges.items():
            if c is not None and c['confidence'] >= .70:
                points = c['measured']
                near = points[points[:, 1] >= np.max(points[:, 1])-.10*np.ptp(points[:, 1]), 0]
                self._curve_tracks[side] = dict(measured=points.copy(), horizontal=c['horizontal'],
                                                near_x=float(np.median(near)))
        return edges

    def _curve_choose_single(self, edges):
        """左线可靠就持续跟左；右线后备可以长期使用，左线重现后再确认身份。"""
        available = [c for c in edges.values() if c is not None]
        left, right = edges['left'], edges['right']
        left_good = left is not None and left['confidence'] >= .70
        right_good = right is not None and right['confidence'] >= .70
        if self.prefer_left_line or self.force_left_line:
            if left_good:
                if self._curve_control_side == 'right' and right_good:
                    self._curve_left_return_frames += 1
                    edge = left if self._curve_left_return_frames >= LANE_LEFT_RETURN_FRAMES else right
                else:
                    edge = left
            else:
                self._curve_left_return_frames = 0
                edge = right if right_good else max(available, key=lambda c: c['confidence'])
        else:
            current = edges.get(self._curve_control_side)
            edge = current if current is not None and current['confidence'] >= .70 else max(available, key=lambda c: c['confidence'])
        self._curve_control_side = edge['side']
        if edge['side'] == 'left':
            self._curve_left_return_frames = 0
        return edge

    def _curve_learn_width(self, edges, mask, y0, width, height):
        """只用同帧双边实测学习 width(y)，补点和单边中点永不参与标定。"""
        left, right = edges['left'], edges['right']
        if (left is None or right is None or left['horizontal'] or right['horizontal']
                or min(left['confidence'], right['confidence']) < .75):
            return
        lp, rp = left['measured'], right['measured']
        rows, li, ri = np.intersect1d(lp[:, 1].astype(int), rp[:, 1].astype(int), return_indices=True)
        widths = rp[ri, 0]-lp[li, 0]
        centers = 0.5*(rp[ri, 0]+lp[li, 0])
        keep = (widths >= .08*width) & (widths <= 1.05*width)
        if np.count_nonzero(keep) < LANE_WIDTH_MIN_ROWS:
            return
        # 车道内部若仍有大面积黄色，通常是同一胶带或场外假配对。
        prefix = np.cumsum(mask[rows] > 0, axis=1)
        lx = np.rint(lp[li, 0]).astype(int)
        rx = np.rint(rp[ri, 0]).astype(int)
        corridor_yellow = (prefix[np.arange(len(rows)), rx]-prefix[np.arange(len(rows)), lx]) / np.maximum(1, widths)
        keep &= corridor_yellow < .25
        ys = (rows+y0)/height
        tolerance = max(4.0, .015*width)
        for _ in range(3):
            if np.count_nonzero(keep) < LANE_WIDTH_MIN_ROWS or np.ptp(ys[keep]) < LANE_WIDTH_MIN_SPAN_RATIO:
                return
            fit = np.polyfit(ys[keep], widths[keep], 1)
            center_fit = np.polyfit(ys[keep], centers[keep], 1)
            new_keep = keep & (np.abs(widths-np.polyval(fit, ys)) <= tolerance)
            new_keep &= np.abs(centers-np.polyval(center_fit, ys)) <= tolerance
            if np.array_equal(new_keep, keep):
                break
            keep = new_keep
        if np.count_nonzero(keep) < LANE_WIDTH_MIN_ROWS or np.ptp(ys[keep]) < LANE_WIDTH_MIN_SPAN_RATIO:
            return
        fit = np.polyfit(ys[keep], widths[keep], 1)
        if not -.02*width <= fit[0] <= 4.0*width:
            return
        fit[0] = max(0.0, fit[0])
        low, high = float(ys[keep].min()), float(ys[keep].max())
        old = self._curve_width_model
        if old is not None:
            common_low, common_high = max(low, old['low']), min(high, old['high'])
            if common_low <= common_high:
                anchors = np.array([common_low, common_high])
                if np.max(np.abs(np.polyval(fit, anchors)-np.polyval(old['fit'], anchors))) > .15*width:
                    return
                fit = (1-LANE_WIDTH_BLEND)*old['fit']+LANE_WIDTH_BLEND*fit
        self._curve_width_model = dict(fit=fit, low=low, high=high)

    def _curve_half_width(self, image_rows, width, height, side):
        """透视先验 + 双边实测模型；坐标以整幅图像为准，与 ROI 裁剪无关。"""
        rows = np.asarray(image_rows, dtype=float)/height
        if self.pre_bridge_guard:
            # 仅桥前采用红线标注的固定半宽；不把图像中轴当作赛道中线。
            prior = 2*width*(PRE_BRIDGE_HALF_WIDTH_SLOPE*rows+PRE_BRIDGE_HALF_WIDTH_INTERCEPT)
        else:
            depth = np.maximum(0.0, (rows-LANE_WIDTH_HORIZON_RATIO)/
                               (LANE_WIDTH_REFERENCE_ROW_RATIO-LANE_WIDTH_HORIZON_RATIO))
            prior = 2*LANE_HALF_RATIO_REFERENCE*width*depth
        prior = np.clip(prior, .08*width, 1.5*width)
        lane_width = prior
        model = self._curve_width_model
        if model is not None:
            distance = np.maximum(model['low']-rows, np.maximum(0.0, rows-model['high']))
            blend = np.clip(1-distance/LANE_WIDTH_EXTRAPOLATION_RATIO, 0, 1)
            learned = np.clip(np.polyval(model['fit'], rows), .08*width, 1.5*width)
            lane_width = blend*learned+(1-blend)*prior
        ratio = self.single_left_half_ratio if side == 'left' else self.single_right_half_ratio
        return .5*lane_width*ratio/LANE_HALF_RATIO_REFERENCE

    @staticmethod
    def _curve_weights(path: np.ndarray, quality: np.ndarray, turn_strength: float):
        """三段各自归一化，补线总权重最多为实测的 1/4。"""
        distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        t = distance / max(1.0, distance[-1])
        bands = np.minimum((3*t).astype(int), 2)
        blend = float(np.clip(turn_strength, 0, 1))
        shares = ((1-blend)*np.array(CURVE_STRAIGHT_WEIGHTS)
                  + blend*np.array(CURVE_SHARP_WEIGHTS))
        weights = np.zeros(len(path), dtype=float)
        for band in range(3):
            selected = bands == band
            total = float(np.sum(quality[selected]))
            if total > 0:
                weights[selected] = shares[band]*quality[selected]/total
        measured = quality >= 0.999
        total_real, total_fill = weights[measured].sum(), weights[~measured].sum()
        if total_fill > CURVE_FILL_TOTAL_RATIO*total_real:
            weights[~measured] *= CURVE_FILL_TOTAL_RATIO*total_real/total_fill
        weights /= max(1e-9, float(weights.sum()))
        effective = tuple(float(weights[bands == band].sum()) for band in range(3))
        return weights, effective

    def _curve_turn_entry(self, selected, x0, y0, width, height, strength):
        """只控制入弯时机；入弯后保持急弯能力，不随画面旋转来回撤销。"""
        if not self.expected_turn_direction:
            self._curve_turn_entry_ready = False
            self._curve_turn_entry_count = 0
            self._curve_turn_entry_frames = 0
            self._curve_exit_armed = False
            self._curve_exit_ready = False
            self._curve_exit_count = 0
            self._curve_exit_frames = 0
            self._turn_ff_strength = 0.0
            return False, 0.0
        if self.turn_entry_hold > 0.0:
            # WAIT 还没跑够 TURN_WAIT_MIN_SECONDS：计数清零，强制停在 WAIT。
            self._curve_turn_entry_count = 0
            self._curve_turn_entry_ready = False
            self._curve_turn_entry_frames = 0
            return False, 0.0
        if self._curve_turn_entry_ready:
            # 入弯后逐帧累计，供方向项软启动使用。
            self._curve_turn_entry_frames += 1
            return True, 0.0
        measured = np.concatenate([c['measured'] for c in selected]) + [x0, y0]
        central = measured[np.abs(measured[:, 0] - 0.5*width)
                           <= TURN_ENTRY_CENTER_HALF_RATIO*width]
        supported = (len(central) >= CURVE_MIN_POINTS
                     and np.ptp(central[:, 0]) >= TURN_ENTRY_MIN_X_SPAN_RATIO*width
                     and len(np.unique(central[:, 0])) >= CURVE_MIN_POINTS)
        row = float(np.median(central[:, 1])) if supported else 0.0
        close = supported and row >= TURN_ENTRY_NEAR_ROW_RATIO*height and strength > 0.0
        self._curve_turn_entry_count = self._curve_turn_entry_count + 1 if close else 0
        self._curve_turn_entry_ready = self._curve_turn_entry_count >= TURN_ENTRY_CONFIRM_FRAMES
        self._curve_turn_entry_frames = 1 if self._curve_turn_entry_ready else 0
        return self._curve_turn_entry_ready, row

    def _detect_columns(self, frame: np.ndarray) -> DetectionResult:
        """连续内沿 + 有限曲线补线。桥上 column_scan=False 时仍走原检测。"""
        height, width = frame.shape[:2]
        debug = np.zeros_like(frame)
        x0, x1 = int(width*self.search_left_ratio), int(width*self.search_right_ratio)
        y0 = int(height*self.search_top_ratio)
        # 急弯内沿常在原 .86 ROI 以下，必须实际看见它，不能靠直线猜测。
        y1 = min(height, int(height*max(self.search_bottom_ratio, CURVE_BOTTOM_RATIO)))
        width_key = (width, height, self.width_profile)
        if width_key != self._curve_width_key:
            self._curve_width_model = None
            self._curve_width_key = width_key
        track_key = (width_key, x0, x1, y0, y1, self.expected_turn_direction,
                     self.prefer_left_line, self.pre_bridge_guard)
        if track_key != self._curve_track_key:
            self._curve_tracks = {}
            self._curve_control_side = None
            self._curve_left_return_frames = 0
            self._curve_turn_entry_ready = False
            self._curve_turn_entry_count = 0
            self._curve_turn_entry_frames = 0
            self._curve_exit_armed = False
            self._curve_exit_ready = False
            self._curve_exit_count = 0
            self._curve_exit_frames = 0
            self._turn_ff_strength = 0.0
            self._curve_track_key = track_key
        self._curve_debug = dict(boundaries=[], center_path=np.empty((0, 2)),
                                 x0=x0, x1=x1, y0=y0, y1=y1)
        if x1 <= x0 or y1 <= y0:
            return DetectionResult(False, 0.0, 0.0, 'empty_roi', debug, curve_mode=True)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._yellow_mask(hsv, local_only=True)[y0:y1, x0:x1].copy()
        if self.morphology_kernel > 1:
            kernel = np.ones((self.morphology_kernel, self.morphology_kernel), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        debug[y0:y1, x0:x1] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        edges = self._curve_boundaries(mask, x0, width, y0, height)
        selected = [c for c in edges.values() if c is not None]
        if not selected:
            self._curve_control_side = None
            self._curve_left_return_frames = 0
            self._curve_turn_entry_count = 0
            reason = 'no_near_boundary' if self.pre_bridge_guard else 'no_yellow_lines'
            return DetectionResult(False, 0.0, 0.0, reason, debug, curve_mode=True)
        self._curve_learn_width(edges, mask, y0, width, height)
        left, right = edges['left'], edges['right']
        path = quality = None
        lane_width = 0.0
        # 只有两条不同的实测边沿，才允许双边配对；优先左线阶段保持该约束。
        if left is not None and right is not None and not (self.force_left_line or self.prefer_left_line):
            if not left['horizontal'] and not right['horizontal']:
                lp, rp = left['path'][::-1], right['path'][::-1]
                common, li, ri = np.intersect1d(lp[:, 1], rp[:, 1], return_indices=True)
                widths = rp[ri, 0]-lp[li, 0]
                usable = ((widths >= self.min_lane_width_ratio*width)
                          & (widths <= self.max_lane_width_ratio*width))
                paired_quality = np.minimum(left['quality'][::-1][li], right['quality'][::-1][ri])
                if np.count_nonzero(usable & (paired_quality >= .999)) >= CURVE_MIN_POINTS:
                    path = np.column_stack(((lp[li[usable], 0]+rp[ri[usable], 0])*0.5, common[usable]))[::-1]
                    quality = np.minimum(left['quality'][::-1][li[usable]], right['quality'][::-1][ri[usable]])[::-1]
                    lane_width = float(np.median(widths[usable]))
        if path is None:
            edge = self._curve_choose_single(edges)
            selected = [edge]
            boundary = edge['path']
            tangent = np.gradient(boundary, axis=0)
            length = np.maximum(1e-6, np.linalg.norm(tangent, axis=1))
            # 法向偏移：竖直边界向车道内侧平移，近水平弯道向画面下方偏移。
            normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))/length[:, None]
            if edge['side'] == 'right':
                normal *= -1
            half = self._curve_half_width(boundary[:, 1]+y0, width, height, edge['side'])
            if self.expected_turn_direction:
                path = boundary + half[:, None]*normal
            else:
                # 平缓路段用同一图像行的宽度恢复中点，避免斜线法向挪动 y 后再错配深度。
                path = boundary.copy()
                path[:, 0] += half if edge['side'] == 'left' else -half
            quality = edge['quality'].copy()
            reason = 'left_line' if edge['side'] == 'left' else 'single_line'
            heading_path = boundary[edge['support']]
        else:
            reason = 'tracking'
            heading_path = path[quality >= 0.999]
        # 图像方向指标，不是米制道路曲率。仅实测支持参与，补线不能制造急弯。
        direction = heading_path[-1]-heading_path[0]
        angle = float(np.degrees(np.arctan2(abs(direction[0]), max(1.0, -direction[1]))))
        turn_strength = float(np.clip((angle-CURVE_HEADING_START_DEG)/
                                      (CURVE_HEADING_FULL_DEG-CURVE_HEADING_START_DEG), 0, 1))
        real_points = sum(len(c['measured']) for c in selected)
        span = max(float(np.linalg.norm(np.ptp(c['measured'], axis=0))) for c in selected)
        rms = max(c['rms'] for c in selected)
        confidence = float(np.clip(0.45*min(1.0, span/(0.30*width))
                                   + 0.30*min(1.0, real_points/100.0)
                                   + 0.25*max(0.0, 1.0-rms/CURVE_MAX_FIT_RMS), 0, 1))
        confidence *= min(c['logic_weight'] for c in selected)
        # 保留弯道中线估计和减速判断；方向增强要等实测弯口到近处。
        stage_strength = turn_strength if self.expected_turn_direction else 0.0
        if confidence >= self.min_confidence:
            turn_ready, turn_row = self._curve_turn_entry(selected, x0, y0, width, height, stage_strength)
        else:
            self._curve_turn_entry_count = 0
            turn_ready, turn_row = self._curve_turn_entry_ready, 0.0
        # 方向前馈按一阶惯性跟随瞬时值，不逐帧相等。
        # 瞬时劲在圆弧变对称 / 左侧直线段漏出时会自己塌掉，一帧归零就等于
        # 出弯判定把转向权威一刀切掉，slope 项（摆正用，硬夹 ±0.30）补不上。
        # 它永远朝瞬时值收敛，所以不是锁死，是动态释放（于萧阳 10.29）。
        ff_target = stage_strength if turn_ready else 0.0
        self._turn_ff_strength += (ff_target - self._turn_ff_strength) * TURN_FF_LAG
        applied_strength = self._turn_ff_strength
        # 方向项软启动：入弯第一帧不能满量程，否则 err 阶跃导致 w 饱和打转。
        heading_ramp = min(1.0, self._curve_turn_entry_frames
                           / float(TURN_HEADING_RAMP_FRAMES))
        heading_error = float(np.sign(direction[0])*applied_strength*heading_ramp)
        # 不把横向弯沿突然改成直线权重，否则位置项会无故向左跳。
        # 入弯前的早转由方向项门控和控制限幅抑制，不篡改当前几何误差。
        weights, effective_weights = self._curve_weights(path, quality, stage_strength)
        target_x = 0.5*width + self.target_offset_pixels
        lane_center = float(np.sum(weights*(path[:, 0]+x0)))
        lateral_error = float((lane_center-target_x)/(0.5*width))
        # ---- 出弯判定：方向项进过弯（strength 到过 0.6）之后又掉到 0.15 以下 ----
        # 并要求车身已回到中线附近，见 TURN_EXIT_MAX_*。bx 仅用于 HUD 观察，不参与判定。
        # measured 是 ROI 局部坐标，须先换算回全图坐标，否则下面的全图比例门限永不成立（于萧阳 10.29）
        box_measured = np.concatenate([c['measured'] for c in selected]) + [x0, y0]
        box_rows = box_measured[(box_measured[:, 1] >= CROSSWALK_TOP_RATIO*height)
                                & (box_measured[:, 1] <= CROSSWALK_BOTTOM_RATIO*height)]
        box_x_max = float(box_rows[:, 0].max()) if len(box_rows) else 0.0
        if self.expected_turn_direction and len(box_rows) >= CURVE_MIN_POINTS:
            # armed 必须是一次性置位。若把兜底条件写在 if 里（不带 not armed），
            # 进弯 3s 后 _curve_turn_entry_frames>=45 会每帧都成立，
            # 于是每帧都把 _curve_exit_count 清零，exit 永远落不了 Y。
            if not self._curve_exit_armed:
                if (stage_strength >= TURN_EXIT_ARM_STRENGTH
                        or self._curve_turn_entry_frames >= TURN_EXIT_ARM_FRAMES):
                    self._curve_exit_armed = True
                    self._curve_exit_count = 0
            elif (stage_strength <= TURN_EXIT_MAX_TURN_STRENGTH
                  and abs(lateral_error) <= TURN_EXIT_MAX_LATERAL):
                self._curve_exit_count += 1
                if self._curve_exit_count >= TURN_EXIT_CONFIRM_FRAMES:
                    self._curve_exit_ready = True
            else:
                self._curve_exit_count = 0
        if self._curve_exit_ready:
            # 出弯后逐帧累计，供斜率项限时使用。
            self._curve_exit_frames += 1
        # 出弯后近端中心线斜率当航向误差：车摆正时斜率≈0，不会像方向项那样变成常值偏置。
        slope_error = 0.0
        if (self._curve_exit_ready and self._curve_exit_frames < TURN_EXIT_SLOPE_FRAMES
                and len(path) >= 12):
            near_path = path[path[:, 1] >= np.percentile(path[:, 1], 55)]
            if len(near_path) >= 8 and float(np.ptp(near_path[:, 1])) >= 8.0:
                slope = float(np.polyfit(near_path[:, 1], near_path[:, 0], 1)[0])
                slope_error = float(np.sign(direction[0])
                                    * np.clip(abs(slope)/TURN_EXIT_SLOPE_REF, 0.0, 1.0))
        slope_term = float(np.clip(TURN_EXIT_SLOPE_GAIN*slope_error,
                                   -TURN_EXIT_SLOPE_MAX, TURN_EXIT_SLOPE_MAX))
        error = float(np.clip(lateral_error + CURVE_HEADING_GAIN*heading_error
                              + slope_term, -1.0, 1.0))

        rejected_points = sum(c['rejected_points'] for c in selected)
        line_rows = {side: sum(len(np.unique(c['measured'][:, 1].astype(int)))
                               for c in selected if c['side'] == side) for side in ('left', 'right')}
        left_x = next((c['near_x'] for c in selected if c['side'] == 'left'), 0.0)
        offset = np.array([x0, y0])
        self._curve_debug.update(boundaries=selected, center_path=path+offset,
                                 point_weights=weights, effective_weights=effective_weights,
                                 target_x=target_x, heading_angle_deg=angle,
                                 raw_turn_strength=turn_strength,
                                 turn_entry_ready=turn_ready, turn_entry_row=turn_row,
                                 width_source=('paired' if self._curve_width_model is not None else
                                               ('mark' if self.pre_bridge_guard else 'prior')),
                                 fill_weight=float(weights[quality < .999].sum()))
        for c in selected:
            # 调试点批量落图，避免每个像素单独调用绘图函数。
            for points, color in ((c['measured'], (0, 220, 0)), (c['rejected'], (0, 0, 255)),
                                  (c['path'][~c['support'] & (np.arange(len(c['path'])) % 4 < 2)], (0, 170, 255))):
                pixels = np.rint(points+offset).astype(int)
                pixels = pixels[(pixels[:, 0] >= 0) & (pixels[:, 0] < width)
                                & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)]
                debug[pixels[:, 1], pixels[:, 0]] = color
        # 画面外的中线不钳制到边缘；只绘制可见点，误差仍用未截断的几何量。
        pixels = np.rint(path[::3]+offset).astype(int)
        pixels = pixels[(pixels[:, 0] >= 0) & (pixels[:, 0] < width)
                        & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)]
        debug[pixels[:, 1], pixels[:, 0]] = (255, 200, 70)
        cv2.line(debug, (int(target_x), y0), (int(target_x), y1-1), (150, 150, 150), 1)
        width_source = ('paired' if self._curve_width_model is not None else
                        ('mark' if self.pre_bridge_guard else 'prior'))
        side_label = 'both' if len(selected) == 2 else selected[0]['side']
        front = 'Y' if self.pre_bridge_guard else 'N'
        cv2.putText(debug, f'pos={lateral_error:+.2f} dir={heading_error:+.2f} width={width_source} side={side_label} front={front}',
                    (8, 110), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1, cv2.LINE_AA)
        if self.expected_turn_direction:
            entry = 'TURN' if turn_ready else 'WAIT'
            row_label = f'{turn_row:.0f}' if turn_row else '-'
            hold = f' hold={self.turn_entry_hold:.1f}s' if self.turn_entry_hold > 0.0 else ''
            cv2.putText(debug, f'entry={entry} row={row_label} need={TURN_ENTRY_NEAR_ROW_RATIO*height:.0f}{hold}'
                               f' exit={"Y" if self._curve_exit_ready else "N"} slope={slope_error:+.2f} bx={box_x_max:.0f}',
                        (8, 127), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 220, 255), 1, cv2.LINE_AA)
        return DetectionResult(confidence >= self.min_confidence, error, confidence, reason, debug,
                               lane_width, line_rows['left'], line_rows['right'], left_x,
                               curve_mode=True, lateral_error=lateral_error, heading_error=heading_error,
                               turn_strength=stage_strength, measured_points=real_points,
                               boundary_span_pixels=span,
                               boundary_side='both' if len(selected) == 2 else selected[0]['side'],
                               band_weights=effective_weights,
                               fill_weight_fraction=float(weights[quality < .999].sum()),
                               logic_rejected_points=rejected_points,
                               turn_prior_applied=bool(self.expected_turn_direction),
                               width_source=width_source,
                               turn_exit_ready=self._curve_exit_ready,
                               heading_slope=slope_error,
                               turn_entry_ready=turn_ready, turn_entry_row=turn_row)


    def detect(self, frame: np.ndarray) -> DetectionResult:
        if self.column_scan:
            return self._detect_columns(frame)
        height, width = frame.shape[:2]
        debug = np.zeros((height, width, 3), dtype=np.uint8)
        x0 = int(width * self.search_left_ratio)
        x1 = int(width * self.search_right_ratio)
        y0 = int(height * self.search_top_ratio)
        y1 = int(height * self.search_bottom_ratio)
        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            return DetectionResult(False, 0.0, 0.0, "empty_roi", debug)

        # 条纹证据先在完整图像中确认，避免搜索框把胶带裁成一小块黄色。
        # 巡线仍只使用原来的矩形搜索范围，没有新增或扩大巡线 ROI。
        full_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = full_hsv[y0:y1, x0:x1]
        mask = self._yellow_mask(full_hsv)[y0:y1, x0:x1].copy()
        if self.morphology_kernel > 1:
            kernel = np.ones(
                (self.morphology_kernel, self.morphology_kernel), dtype=np.uint8
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        roi_height, roi_width = mask.shape
        min_line_width = max(2, int(roi_width * self.min_line_width_ratio))
        max_line_width = max(
            min_line_width + 1, int(roi_width * self.max_line_width_ratio)
        )
        expected_line_width = roi_width * self.expected_line_width_ratio
        line_tolerance = max(1.0, roi_width * self.width_tolerance_ratio)
        min_lane_width = roi_width * self.min_lane_width_ratio
        max_lane_width = roi_width * self.max_lane_width_ratio
        expected_lane_width = roi_width * self.expected_lane_width_ratio
        lane_tolerance = max(1.0, roi_width * self.lane_width_tolerance_ratio)
        max_jump_per_row = max(1.0, roi_width * self.max_center_jump_ratio)
        target_x = 0.5 * width + self.target_offset_pixels

        left_mask = np.zeros_like(mask)
        right_mask = np.zeros_like(mask)
        lane_centers: List[float] = []
        lane_weights: List[float] = []
        lane_center_points: List[Tuple[float, int]] = []
        lane_widths: List[float] = []
        boundary_widths: List[float] = []
        boundary_lane_contrasts: List[float] = []
        previous_center: Optional[float] = None
        previous_row: Optional[int] = None
        pair_rows = 0
        single_rows = 0

        # 批量提取所有行的黄色区间，保持从下到上的跟踪顺序及配对规则。
        padded_mask = np.zeros((roi_height, roi_width + 2), dtype=np.int8)
        padded_mask[:, 1:-1] = mask > 0
        changes = np.diff(padded_mask, axis=1)
        run_rows, run_starts = np.nonzero(changes == 1)
        _, run_ends = np.nonzero(changes == -1)
        run_widths = run_ends - run_starts
        keep = (run_widths >= min_line_width) & (run_widths <= max_line_width)
        row_runs: List[List[Tuple[int, int, int]]] = [[] for _ in range(roi_height)]
        for row_y, start, end, run_width in zip(
            run_rows[keep].tolist(), run_starts[keep].tolist(),
            run_ends[keep].tolist(), run_widths[keep].tolist(),
        ):
            row_runs[row_y].append((start, end, run_width))

        for row_y in range(roi_height - 1, -1, -1):
            runs = row_runs[row_y]
            if not runs:
                continue
            # 近处还没锁到线时，不要用画面上部的场外黄带当赛道。
            if previous_center is None and row_y < int(
                roi_height * self.near_lock_ratio
            ):
                continue

            reference = previous_center if previous_center is not None else target_x
            row_gap = 1 if previous_row is None else max(1, previous_row - row_y)
            allowed_jump = min(0.38 * roi_width, max_jump_per_row * row_gap)
            center = None
            lane_width = 0.0
            contrast = 0.0
            weight = 1.0 + 1.4 * (row_y / max(1, roi_height - 1))
            used_pair = False

            if self.force_left_line:
                start, end, run_width = min(runs, key=lambda item: item[0])
                half_lane = self.single_left_half_ratio * width
                candidate = x0 + float(end - 1) + half_lane
                candidate = float(np.clip(candidate, x0 + 8, x1 - 8))
                if previous_center is None or abs(candidate - previous_center) <= (
                    1.35 * allowed_jump
                ):
                    center = candidate
                    lane_width = expected_lane_width
                    weight *= 0.70
                    left_mask[row_y, start:end] = 255
                    boundary_widths.append(run_width)
            elif len(runs) >= 2:
                best_pair = None
                for left_idx, left in enumerate(runs[:-1]):
                    for right in runs[left_idx + 1 :]:
                        left_inner = x0 + float(left[1] - 1)
                        right_inner = x0 + float(right[0])
                        pair_width = right_inner - left_inner
                        if not (min_lane_width <= pair_width <= max_lane_width):
                            continue
                        candidate = 0.5 * (left_inner + right_inner)
                        if (
                            previous_center is not None
                            and abs(candidate - previous_center) > allowed_jump
                        ):
                            continue
                        pair_contrast = 0.0
                        corridor = hsv[row_y, left[1] : right[0], 1]
                        if corridor.size > 0:
                            boundary_sat = 0.5 * (
                                float(np.mean(hsv[row_y, left[0] : left[1], 1]))
                                + float(np.mean(hsv[row_y, right[0] : right[1], 1]))
                            )
                            pair_contrast = boundary_sat - float(
                                np.median(corridor)
                            )
                        if pair_contrast < self.min_boundary_lane_contrast:
                            continue
                        width_err = abs(pair_width - expected_lane_width)
                        if best_pair is None or width_err < best_pair[0]:
                            best_pair = (
                                width_err,
                                left,
                                right,
                                pair_width,
                                pair_contrast,
                                candidate,
                            )
                if best_pair is not None:
                    (
                        _width_err,
                        left,
                        right,
                        lane_width,
                        contrast,
                        candidate,
                    ) = best_pair
                    center = candidate
                    used_pair = True
                    left_mask[row_y, left[0] : left[1]] = 255
                    right_mask[row_y, right[0] : right[1]] = 255
                    boundary_widths.extend((left[2], right[2]))

            if center is None and not self.force_left_line and len(runs) >= 1:
                start, end, run_width = (
                    min(runs, key=lambda item: abs(x0 + 0.5 * (item[0] + item[1]) - reference))
                )
                run_center = x0 + 0.5 * (start + end - 1)
                if run_center <= target_x:
                    # 只看到左黄线：用左内沿加上假定半宽，得到虚拟赛道中点。
                    half_lane = self.single_left_half_ratio * width
                    candidate = x0 + float(end - 1) + half_lane
                    left_mask[row_y, start:end] = 255
                else:
                    # 只看到右黄线：用右内沿减去假定半宽。
                    half_lane = self.single_right_half_ratio * width
                    candidate = x0 + float(start) - half_lane
                    right_mask[row_y, start:end] = 255
                candidate = float(np.clip(candidate, x0 + 8, x1 - 8))
                if previous_center is None or abs(candidate - previous_center) <= (
                    1.35 * allowed_jump
                ):
                    center = candidate
                    lane_width = expected_lane_width
                    contrast = max(contrast, 0.0)
                    weight *= 0.55
                    boundary_widths.append(run_width)

            if center is None:
                continue
            if used_pair:
                pair_rows += 1
            else:
                single_rows += 1
            lane_centers.append(center)
            lane_weights.append(weight)
            lane_center_points.append((center, row_y))
            lane_widths.append(lane_width)
            boundary_lane_contrasts.append(contrast)
            previous_center = center
            previous_row = row_y

        debug[y0:y1, x0:x1] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(debug, (x0, y0), (x1, y1), (80, 80, 80), 1)
        cv2.line(
            debug,
            (int(target_x), y0),
            (int(target_x), y1),
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )

        mean_lane_width = float(np.mean(lane_widths)) if lane_widths else 0.0
        left_line_rows = int(np.count_nonzero(np.any(left_mask > 0, axis=1)))
        right_line_rows = int(np.count_nonzero(np.any(right_mask > 0, axis=1)))
        left_rows_present = np.any(left_mask > 0, axis=1)
        if np.any(left_rows_present):
            rightmost = roi_width - 1 - np.argmax(
                left_mask[left_rows_present, ::-1] > 0, axis=1
            )
            left_edge_x = float(np.mean(x0 + rightmost.astype(np.float64)))
        else:
            left_edge_x = 0.0
        mean_contrast = (
            float(np.mean(boundary_lane_contrasts))
            if boundary_lane_contrasts
            else 0.0
        )
        cv2.putText(
            debug,
            f"L{left_line_rows} R{right_line_rows}  "
            f"lx={left_edge_x:.0f}  "
            f"pairs={pair_rows} single={single_rows}  "
            f"lane={mean_lane_width:.0f}px",
            (max(8, x0 + 6), min(debug.shape[0] - 10, y1 + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )

        if not lane_centers:
            return DetectionResult(
                False,
                0.0,
                0.0,
                "no_yellow_lines",
                debug,
                0.0,
                left_line_rows,
                right_line_rows,
                left_edge_x,
            )

        pair_coverage = pair_rows / roi_height
        single_coverage = single_rows / roi_height
        coverage = pair_coverage + 0.55 * single_coverage
        line_quality = float(
            np.mean(
                [
                    max(0.0, 1.0 - abs(value - expected_line_width) / line_tolerance)
                    for value in boundary_widths
                ]
            )
        ) if boundary_widths else 0.0
        lane_quality = float(
            np.mean(
                [
                    max(0.0, 1.0 - abs(value - expected_lane_width) / lane_tolerance)
                    for value in lane_widths
                ]
            )
        )
        if len(lane_centers) > 1:
            continuity = float(
                np.mean(
                    np.maximum(
                        0.0,
                        1.0 - np.abs(np.diff(lane_centers)) / max_jump_per_row,
                    )
                )
            )
        else:
            continuity = 0.0
        confidence = (
            0.45 * min(1.0, coverage / max(self.min_valid_row_fraction, 0.12))
            + 0.20 * line_quality
            + 0.20 * lane_quality
            + 0.15 * continuity
        )
        lane_center = float(
            np.average(lane_centers, weights=np.array(lane_weights, dtype=float))
        )
        error = float(
            np.clip((lane_center - target_x) / (0.5 * width), -1.0, 1.0)
        )

        look_y = y0 + int(0.62 * roi_height)
        if self._hud_cx is None:
            self._hud_cx = lane_center
            self._hud_cy = float(look_y)
        else:
            self._hud_cx = 0.78 * self._hud_cx + 0.22 * lane_center
            self._hud_cy = 0.78 * self._hud_cy + 0.22 * look_y
        cx, cy = int(self._hud_cx), int(self._hud_cy)
        cv2.line(debug, (cx - 14, cy), (cx + 14, cy), (220, 220, 220), 1, cv2.LINE_AA)
        cv2.line(debug, (cx, cy - 14), (cx, cy + 14), (220, 220, 220), 1, cv2.LINE_AA)
        cv2.circle(debug, (cx, cy), 8, (255, 255, 255), 2, cv2.LINE_AA)

        enough_pair = pair_coverage >= self.min_valid_row_fraction
        enough_single = single_coverage >= self.min_single_coverage
        enough_mix = coverage >= self.min_valid_row_fraction
        if not (enough_pair or enough_single or enough_mix):
            return DetectionResult(
                False,
                error,
                confidence,
                "insufficient_double_line",
                debug,
                mean_lane_width,
                left_line_rows,
                right_line_rows,
                left_edge_x,
            )
        if confidence < self.min_confidence:
            return DetectionResult(
                False,
                error,
                confidence,
                "low_confidence",
                debug,
                mean_lane_width,
                left_line_rows,
                right_line_rows,
                left_edge_x,
            )
        reason = "left_line" if self.force_left_line else (
            "tracking" if pair_rows > 0 else "single_line"
        )
        return DetectionResult(
            True,
            error,
            confidence,
            reason,
            debug,
            mean_lane_width,
            left_line_rows,
            right_line_rows,
            left_edge_x,
        )


class TrackSceneDetector:
    """识别人行道斑马线和黄黑相间警戒胶带。"""

    def __init__(
        self,
        yellow_h_min: int,
        yellow_h_max: int,
        yellow_s_min: int,
        yellow_v_min: int,
        black_value_max: int,
        black_saturation_max: int,
        crosswalk_top_ratio: float,
        crosswalk_bottom_ratio: float,
        hazard_top_ratio: float,
        hazard_bottom_ratio: float,
        side_margin_ratio: float,
        min_crosswalk_stripes: int,
        min_hazard_transitions: float,
    ) -> None:
        self.yellow_h_min = yellow_h_min
        self.yellow_h_max = yellow_h_max
        self.yellow_s_min = yellow_s_min
        self.yellow_v_min = yellow_v_min
        self.black_value_max = black_value_max
        self.black_saturation_max = black_saturation_max
        self.crosswalk_top_ratio = crosswalk_top_ratio
        self.crosswalk_bottom_ratio = crosswalk_bottom_ratio
        self.hazard_top_ratio = hazard_top_ratio
        self.hazard_bottom_ratio = hazard_bottom_ratio
        self.side_margin_ratio = side_margin_ratio
        self.min_crosswalk_stripes = min_crosswalk_stripes
        self.min_hazard_transitions = min_hazard_transitions
        # 框内允许的黄像素占比。直接取模块常量，不走构造参数：
        # 它只在 _detect_crosswalk 里用，不需要 YAML 暴露。
        # 注意：必须在 TrackSceneDetector.__init__ 里赋，不能只在节点的
        # _apply_field_tuning 里赋，否则 _detect_crosswalk 拿不到（AttributeError）。
        self.crosswalk_max_yellow_frac = float(CROSSWALK_MAX_YELLOW_FRAC)

    def _masks(self, hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        yellow = cv2.inRange(
            hsv,
            np.array(
                (self.yellow_h_min, self.yellow_s_min, self.yellow_v_min),
                dtype=np.uint8,
            ),
            np.array((self.yellow_h_max, 255, 255), dtype=np.uint8),
        )
        black = cv2.inRange(
            hsv,
            np.array((0, 0, 0), dtype=np.uint8),
            np.array(
                (180, self.black_saturation_max, self.black_value_max),
                dtype=np.uint8,
            ),
        )
        black = cv2.bitwise_and(black, cv2.bitwise_not(yellow))
        return yellow, black

    @staticmethod
    def _peak_count(signal: np.ndarray, threshold: float, min_gap: int) -> int:
        above = signal >= threshold
        padded = np.pad(above.astype(np.int8), (1, 1))
        changes = np.diff(padded)
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)
        count = 0
        last_center = -1e9
        for start, end in zip(starts.tolist(), ends.tolist()):
            if end - start < 2:
                continue
            center = 0.5 * (start + end)
            if center - last_center >= min_gap:
                count += 1
                last_center = center
        return count

    def detect(
        self,
        frame: np.ndarray,
        debug: np.ndarray,
        prefer_finish: bool = False,
    ) -> SceneObservation:
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        margin = int(width * self.side_margin_ratio)

        cw_y0 = int(height * self.crosswalk_top_ratio)
        cw_y1 = int(height * self.crosswalk_bottom_ratio)
        hz_y0 = int(height * self.hazard_top_ratio)
        hz_y1 = int(height * self.hazard_bottom_ratio)
        x0 = margin
        x1 = max(x0 + 8, width - margin)

        crosswalk, stripe_count, cw_score, cw_mask = self._detect_crosswalk(
            hsv[cw_y0:cw_y1, x0:x1]
        )
        fn_y0 = int(height * FINISH_TOP_RATIO)
        fn_y1 = int(height * FINISH_BOTTOM_RATIO)
        finish, blob_count, fn_score, blob_boxes = self._detect_finish(
            hsv[fn_y0:fn_y1, x0:x1]
        )
        # 过完隧道才认终点。之前斑马线会被大字逻辑抢走，人行道就停不了。
        if not prefer_finish:
            finish = False
        elif finish:
            crosswalk = False
        hazard, hz_score = self._detect_hazard(hsv[hz_y0:hz_y1, x0:x1])
        near_y0 = int(height * BRIDGE_NEAR_TOP_RATIO)
        near_y1 = int(height * BRIDGE_NEAR_BOTTOM_RATIO)
        hazard_near, _ = self._detect_hazard(hsv[near_y0:near_y1, x0:x1])

        roi = debug[cw_y0:cw_y1, x0:x1]
        if cw_mask.size > 1 and roi.size > 0 and cw_mask.shape[:2] == roi.shape[:2]:
            stripe = cv2.cvtColor(cw_mask, cv2.COLOR_GRAY2BGR)
            debug[cw_y0:cw_y1, x0:x1] = np.maximum(roi, stripe)
        box_tone = 220 if crosswalk else 90
        cv2.rectangle(debug, (x0, cw_y0), (x1, cw_y1), (box_tone, box_tone, box_tone), 1)
        cv2.putText(
            debug,
            f"CW {stripe_count}" + (" HIT" if crosswalk else f" {self.cw_reason}"),
            (x0 + 8, min(height - 12, cw_y0 + 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255) if crosswalk else (170, 170, 170),
            1,
            cv2.LINE_AA,
        )

        if prefer_finish:
            fn_tone = 220 if finish else 90
            cv2.rectangle(
                debug, (x0, fn_y0), (x1, fn_y1), (fn_tone, fn_tone, fn_tone), 1
            )
            for bx, by, bw, bh in blob_boxes:
                cv2.rectangle(
                    debug,
                    (x0 + bx, fn_y0 + by),
                    (x0 + bx + bw, fn_y0 + by + bh),
                    (220, 220, 220),
                    1,
                )
            cv2.putText(
                debug,
                f"FN {blob_count}" + (" HIT" if finish else ""),
                (x0 + 8, min(height - 12, fn_y0 + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255) if finish else (170, 170, 170),
                1,
                cv2.LINE_AA,
            )
        if hazard:
            cv2.putText(
                debug,
                "HAZARD",
                (width - 110, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if hazard_near:
            cv2.putText(
                debug,
                "NEAR",
                (width - 110, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
        return SceneObservation(
            crosswalk=crosswalk,
            hazard=hazard,
            hazard_near=hazard_near,
            finish=finish,
            crosswalk_score=cw_score,
            hazard_score=hz_score,
            finish_score=fn_score,
            stripe_count=stripe_count,
            finish_blobs=blob_count,
        )

    def _detect_finish(
        self, hsv_roi: np.ndarray
    ) -> Tuple[bool, int, float, List[Tuple[int, int, int, int]]]:
        empty: List[Tuple[int, int, int, int]] = []
        if hsv_roi.size == 0:
            return False, 0, 0.0, empty
        yellow, black = self._masks(hsv_roi)
        close_kernel = np.ones((11, 11), dtype=np.uint8)
        open_kernel = np.ones((5, 5), dtype=np.uint8)
        black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, close_kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, open_kernel)
        yellow_frac = float(np.mean(yellow > 0))
        black_frac = float(np.mean(black > 0))
        if black_frac < 0.07 or black_frac > 0.62 or yellow_frac > 0.20:
            return False, 0, 0.0, empty

        binary = (black > 0).astype(np.uint8)
        roi_h, roi_w = binary.shape
        near = binary[int(FINISH_NEAR_RATIO * roi_h) :, :]
        if near.size == 0 or float(np.mean(near)) < 0.08:
            return False, 0, 0.0, empty
        num, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        min_area = 0.012 * roi_h * roi_w
        blobs: List[Tuple[int, int, int, int, int]] = []
        for index in range(1, num):
            x = int(stats[index, cv2.CC_STAT_LEFT])
            y = int(stats[index, cv2.CC_STAT_TOP])
            w = int(stats[index, cv2.CC_STAT_WIDTH])
            h = int(stats[index, cv2.CC_STAT_HEIGHT])
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < min_area or h < 0.20 * roi_h or w < 0.06 * roi_w:
                continue
            if w > 3.2 * max(h, 1):
                continue
            blobs.append((x, y, w, h, area))

        blob_count = len(blobs)
        boxes = [(item[0], item[1], item[2], item[3]) for item in blobs]
        if blob_count == 1:
            _x, _y, w, h, area = blobs[0]
            if w < 0.42 * roi_w or h < 0.26 * roi_h or area < 0.08 * roi_h * roi_w:
                return False, blob_count, 0.0, boxes
        elif blob_count < 2 or blob_count > 6:
            return False, blob_count, 0.0, boxes

        left = min(item[0] for item in blobs)
        right = max(item[0] + item[2] for item in blobs)
        span = (right - left) / max(1, roi_w)
        if span < 0.38:
            return False, blob_count, 0.0, boxes
        bottoms = [item[1] + item[3] for item in blobs]
        if max(bottoms) < FINISH_NEAR_RATIO * roi_h:
            return False, blob_count, 0.0, boxes

        aspects = [item[2] / max(item[3], 1) for item in blobs]
        median_aspect = float(np.median(np.array(aspects, dtype=np.float32)))
        if median_aspect > 2.4:
            return False, blob_count, 0.0, boxes

        score = float(
            np.clip(
                0.40 * min(1.0, blob_count / 3.0)
                + 0.35 * min(1.0, span / 0.70)
                + 0.25 * min(1.0, black_frac / 0.28),
                0.0,
                1.0,
            )
        )
        if blob_count == 1:
            score = min(1.0, score + 0.08)
        return score >= 0.40, blob_count, score, boxes

    def _detect_crosswalk(
        self, hsv_roi: np.ndarray
    ) -> Tuple[bool, int, float, np.ndarray]:
        empty = np.zeros((1, 1), dtype=np.uint8)
        if hsv_roi.size == 0:
            return False, 0, 0.0, empty
        yellow, black = self._masks(hsv_roi)
        kernel = np.ones((3, 5), dtype=np.uint8)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, kernel)
        black_bin = black > 0
        yellow_frac = float(np.mean(yellow > 0))
        black_frac = float(np.mean(black_bin))
        # cw_reason 只用于 HUD：以前所有 early return 都把 stripe_count 写死 0，
        # 画面上明明有 6 条也只会显示 CW 0，根本看不出是哪一道门拒的。
        self.cw_reason = f'b={black_frac:.2f} y={yellow_frac:.2f}'
        if (black_frac < 0.06 or black_frac > 0.58
                or yellow_frac > self.crosswalk_max_yellow_frac):
            return False, 0, 0.0, black
        # 斑马线必须落到画面下部（靠近车），只看见远处条纹不算进入停车区。
        roi_h, roi_w = black_bin.shape
        near = black_bin[int(0.55 * roi_h) :, :]
        if near.size == 0 or float(np.mean(near)) < 0.10:
            self.cw_reason += ' near'
            return False, 0, 0.0, black

        row_fill = black_bin.mean(axis=1)
        col_fill = black_bin.mean(axis=0)
        row_peaks = self._peak_count(row_fill, 0.18, max(3, roi_h // 18))
        col_peaks = self._peak_count(col_fill, 0.16, max(4, roi_w // 16))
        stripe_count = max(row_peaks, col_peaks)
        self.cw_reason += f' r{row_peaks} c{col_peaks}'
        if stripe_count < self.min_crosswalk_stripes:
            return False, stripe_count, 0.0, black

        span_ok = False
        if row_peaks >= self.min_crosswalk_stripes:
            active = np.flatnonzero(row_fill >= 0.18)
            if (
                active.size
                and (active[-1] - active[0]) >= 0.18 * roi_h
                and active[-1] >= 0.62 * roi_h
            ):
                span_ok = True
        if col_peaks >= self.min_crosswalk_stripes:
            active = np.flatnonzero(col_fill >= 0.16)
            if active.size and (active[-1] - active[0]) >= 0.28 * roi_w:
                span_ok = True
        if not span_ok:
            self.cw_reason += ' span'
            return False, stripe_count, 0.0, black

        score = float(
            np.clip(
                0.45 * min(1.0, stripe_count / 6.0)
                + 0.35 * min(1.0, black_frac / 0.28)
                + 0.20 * max(0.0, 1.0 - yellow_frac / 0.16),
                0.0,
                1.0,
            )
        )
        return score >= 0.42, stripe_count, score, black

    def _yellow_black_transitions(
        self, yellow_bin: np.ndarray, black_bin: np.ndarray, axis: int
    ) -> List[float]:
        length = yellow_bin.shape[axis]
        width = yellow_bin.shape[1 - axis]
        step = max(1, length // 28)
        window = max(24, int(0.30 * width))
        transitions: List[float] = []
        for index in range(0, length, step):
            if axis == 0:
                y_line = yellow_bin[index]
                b_line = black_bin[index]
            else:
                y_line = yellow_bin[:, index]
                b_line = black_bin[:, index]
            state = np.where(y_line, 1, np.where(b_line, 2, 0)).astype(np.int8)
            compact_idx = np.flatnonzero(state)
            if compact_idx.size < 4:
                continue
            compact = state[compact_idx]
            pair = compact[:-1] + compact[1:]
            changed = compact[1:] != compact[:-1]
            flips = changed & (pair == 3)
            if not np.any(flips):
                continue
            flip_x = compact_idx[1:][flips]
            local = 1
            start = 0
            for end in range(len(flip_x)):
                while int(flip_x[end] - flip_x[start]) > window:
                    start += 1
                local = max(local, end - start + 1)
            if local >= 2:
                transitions.append(float(local))
        return transitions

    def _detect_hazard(self, hsv_roi: np.ndarray) -> Tuple[bool, float]:
        if hsv_roi.size == 0:
            return False, 0.0
        yellow, black = self._masks(hsv_roi)
        kernel = np.ones((3, 3), dtype=np.uint8)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
        yellow_bin = yellow > 0
        black_bin = black > 0
        yellow_frac = float(np.mean(yellow_bin))
        black_frac = float(np.mean(black_bin))
        if yellow_frac < 0.018 or black_frac < 0.010:
            return False, 0.0

        dilated_yellow = cv2.dilate(yellow, np.ones((7, 7), dtype=np.uint8))
        adjacent = float(np.mean((dilated_yellow > 0) & black_bin))
        if adjacent < 0.004:
            return False, 0.0

        row_transitions = self._yellow_black_transitions(yellow_bin, black_bin, 0)
        col_transitions = self._yellow_black_transitions(yellow_bin, black_bin, 1)
        mean_row = float(np.mean(row_transitions)) if row_transitions else 0.0
        mean_col = float(np.mean(col_transitions)) if col_transitions else 0.0
        mean_transitions = max(mean_row, mean_col)
        support = max(len(row_transitions), len(col_transitions))
        if mean_transitions < self.min_hazard_transitions or support < 6:
            return False, 0.0

        score = float(
            np.clip(
                0.40 * min(1.0, mean_transitions / 6.0)
                + 0.35 * min(1.0, adjacent / 0.06)
                + 0.25 * min(1.0, (yellow_frac + black_frac) / 0.20),
                0.0,
                1.0,
            )
        )
        return score >= 0.32, score


class BridgeTraversal:
    """纯状态机：视觉预告，姿态确认上桥，回平 + 前进距离确认离桥。

    不发布 ROS 命令。所有时间是本机 monotonic，距离来自轮式里程速度积分，
    只用作附加约束，不能将其当作无打滑的真实地面距离。
    """
    def __init__(self):
        self.state = "idle"
        self.reason = ""
        self.armed_at = None
        self.near = False
        self.imu_at = None
        self.odom_at = None
        self.roll = self.pitch = self.yaw = 0.0
        self.roll_rate = self.pitch_rate = 0.0
        self.speed = 0.0
        self.distance = 0.0
        self.samples = deque(maxlen=120)
        self.baseline = None
        self.tilt_since = None
        self.entered_at = None
        self.entered_distance = 0.0
        self.heading = 0.0
        self.peak_roll = 0.0
        self.flat_since = None
        self.flat_distance = 0.0
        self.exit_good_seconds = 0.0
        self.exit_sample_at = None
        self.exit_last_good = False
        self.exit_retilt_since = None
        self.exit_hold_since = None
        self.last_progress_at = None
        self.progress_distance = 0.0

    @staticmethod
    def angle_delta(a, b):
        return (a - b + 180.0) % 360.0 - 180.0

    @property
    def crossing(self):
        return self.state in ("crossing", "exit_slow", "exit_hold")

    @property
    def boost(self):
        return self.state == "crossing" or (
            self.state == "approach" and self.near and self.baseline is not None
        )

    def imu(self, q, gyro, now):
        if not all(math.isfinite(v) for v in (*q, *gyro)):
            return
        norm = math.sqrt(sum(v * v for v in q))
        if not 0.8 <= norm <= 1.2:
            return
        w, x, y, z = (v / norm for v in q)
        self.roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))))
        self.yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        self.roll_rate, self.pitch_rate = (math.degrees(v) for v in gyro[:2])
        self.imu_at = now
        # 只在尚未确定基准时标定；上桥后不能把持续倾斜学成“水平”。
        if self.baseline is None and self.state in ("idle", "approach"):
            if max(abs(self.roll_rate), abs(self.pitch_rate)) > BRIDGE_BASE_RATE_DEG:
                self.samples.clear()
                return
            self.samples.append((now, self.roll, self.pitch))
            while self.samples and now - self.samples[0][0] > 0.9:
                self.samples.popleft()
            if (len(self.samples) >= 10 and
                    now - self.samples[0][0] >= BRIDGE_BASE_SECONDS):
                rolls = [s[1] for s in self.samples]
                pitches = [s[2] for s in self.samples]
                if max(rolls)-min(rolls) < 1.5 and max(pitches)-min(pitches) < 2.5:
                    self.baseline = (sum(rolls)/len(rolls), sum(pitches)/len(pitches))

    def odom(self, speed, now):
        if not math.isfinite(speed) or abs(speed) > 1.5:
            return
        if self.odom_at is not None:
            dt = now - self.odom_at
            if 0.0 < dt <= BRIDGE_SENSOR_TIMEOUT:
                self.distance += max(0.0, 0.5 * (self.speed + speed)) * dt
        self.speed, self.odom_at = speed, now

    def fresh(self, now):
        return all(t is not None and 0 <= now-t <= BRIDGE_SENSOR_TIMEOUT
                   for t in (self.imu_at, self.odom_at))

    def arm(self, now):
        if self.state == "idle":
            self.state, self.armed_at = "approach", now

    def fail(self, reason):
        if self.state not in ("done", "fault"):
            self.state, self.reason = "fault", reason

    def tick(self, now):
        if self.state in ("idle", "done", "fault"):
            return
        if not self.fresh(now) or self.baseline is None:
            self.tilt_since = self.flat_since = None
            if self.crossing or self.boost or now-self.armed_at > BRIDGE_SENSOR_START_GRACE:
                self.fail("IMU/odom stale or level baseline unavailable")
            return
        dr = abs(self.angle_delta(self.roll, self.baseline[0]))
        dp = abs(self.angle_delta(self.pitch, self.baseline[1]))
        if self.state == "approach":
            if dr >= BRIDGE_ENTER_ROLL_DEG:
                if self.tilt_since is None:
                    self.tilt_since = self.imu_at
                if (self.imu_at-self.tilt_since >= BRIDGE_ENTER_CONFIRM_SECONDS
                        and self.speed >= BRIDGE_PROGRESS_SPEED):
                    self.state = "crossing"
                    self.entered_at = now
                    self.entered_distance = self.progress_distance = self.distance
                    self.last_progress_at = now
                    self.heading = self.yaw
                    self.peak_roll = dr
            else:
                self.tilt_since = None
            if self.state == "approach" and now-self.armed_at > BRIDGE_APPROACH_TIMEOUT:
                self.fail("visual bridge not confirmed by body roll")
            return
        self.peak_roll = max(self.peak_roll, dr)
        if self.distance-self.progress_distance >= 0.02:
            self.progress_distance, self.last_progress_at = self.distance, now
        if self.state == "crossing" and now-self.last_progress_at > BRIDGE_STALL_TIMEOUT:
            self.fail("no odometry progress on bridge")
            return
        if self.state == "crossing" and now-self.entered_at > BRIDGE_CROSS_TIMEOUT:
            self.fail("bridge exit not confirmed before timeout")
            return
        if self.state == "crossing":
            returned = (
                dr <= BRIDGE_EXIT_START_ROLL_DEG
                and dr <= self.peak_roll * BRIDGE_EXIT_RETURN_RATIO
                and self.peak_roll-dr >= BRIDGE_RETURN_DELTA_DEG
                and self.speed >= BRIDGE_PROGRESS_SPEED
                and self.distance-self.entered_distance >= BRIDGE_MIN_DISTANCE
                and now-self.entered_at >= BRIDGE_MIN_CROSS_SECONDS
            )
            if not returned:
                return
            # 先降速再确认，不等待角速度、俯仰同时安静才松开冲桥速度。
            # 锁定此阶段；后续只能 done/fault，不能重新进入 crossing。
            self.state, self.flat_since = "exit_slow", now
            self.flat_distance = self.distance
            self.exit_sample_at = self.imu_at
            self.exit_good_seconds = 0.0
            self.exit_last_good = False
            self.exit_retilt_since = None

        good = (
            dr <= BRIDGE_EXIT_ROLL_DEG and dp <= BRIDGE_EXIT_PITCH_DEG
            and abs(self.roll_rate) <= BRIDGE_EXIT_ROLL_RATE_DEG
            and abs(self.pitch_rate) <= BRIDGE_EXIT_PITCH_RATE_DEG
        )
        # 同一帧 IMU 被控制循环重复读取，不累计稳定时间。
        dt = self.imu_at-self.exit_sample_at
        if dt > 0.0:
            if good and self.exit_last_good and dt <= 0.20:
                self.exit_good_seconds += dt
            # 短暂坏样本暂停累计，不清空整个离桥阶段，也不恢复冲桥。
            self.exit_last_good = good
            self.exit_sample_at = self.imu_at
        retilted = (dr >= BRIDGE_EXIT_RETILT_ROLL_DEG
                    or dp >= BRIDGE_EXIT_RETILT_PITCH_DEG)
        if retilted:
            if self.exit_retilt_since is None:
                self.exit_retilt_since = self.imu_at
            if self.imu_at-self.exit_retilt_since >= BRIDGE_EXIT_RETILT_SECONDS:
                self.fail("body tilted again during slow exit; bridge not cleared")
                return
        else:
            self.exit_retilt_since = None
        if (good and self.exit_good_seconds >= BRIDGE_EXIT_STABLE_SECONDS
                and self.distance-self.flat_distance >= BRIDGE_EXIT_CLEARANCE):
            self.state = "done"
            return
        # 先给本帧有效回平证据完成确认的机会，再限制继续前进。
        # 下桥轮速瞬变会使里程预算很快耗尽，这不是永久故障的充分证据。
        if self.state == "exit_slow" and (
                now-self.flat_since >= BRIDGE_EXIT_TIMEOUT
                or self.distance-self.flat_distance >= BRIDGE_EXIT_MAX_DISTANCE):
            self.state, self.exit_hold_since = "exit_hold", now
            self.reason = "exit travel budget reached; stopping to confirm level"
        if (self.state == "exit_hold"
                and now-self.exit_hold_since >= BRIDGE_EXIT_SETTLE_TIMEOUT):
            self.fail("body level/clearance still unconfirmed after exit hold")


class ShiWaiFollower(Node):
    VALID_ROTATIONS = (0, 90, 180, 270)
    VALID_MIRRORS = ("none", "horizontal", "vertical", "both")
    MODE_FOLLOW = "FOLLOW"
    MODE_STOP = "SIDEWALK_STOP"
    MODE_CROUCH = "TUNNEL_CROUCH"
    MODE_FINISH = "FINISH_STOP"

    def __init__(self) -> None:
        super().__init__("shiwai")
        self._declare_parameters()
        self._load_parameters()
        self._apply_field_tuning()
        self._validate_parameters()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self._bridge = BridgeTraversal()
        self._bridge_announced_state = "idle"
        self._last_imu_stamp = self._last_odom_stamp = None
        self._post_bridge_samples = deque(maxlen=32)
        self._post_bridge_last_sample = None
        self._post_bridge_speed_mode = "wait"
        self._post_bridge_speed_target = 0.0
        self._imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, qos_profile_sensor_data
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, qos_profile_sensor_data
        )
        self._frame_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._stop_capture = threading.Event()
        self._frame: Optional[np.ndarray] = None
        self._frame_time = 0.0
        self._sequence = 0
        self._camera_ok = False
        self._capture: Optional[cv2.VideoCapture] = None
        self._last_sequence = -1
        self._last_command = (0.0, 0.0)
        self._last_command_valid = False
        self._last_motion_time = time.monotonic()
        self._previous_error = 0.0
        self._previous_control_time = 0.0
        self._have_error = False
        self._last_log = 0.0
        self._last_status = 0.0
        self._exit_requested = False
        self._closing = False
        self._window_ready = False

        self._mode = self.MODE_FOLLOW
        self._passed_bridge = False
        self._passed_sidewalk = False
        self._passed_tunnel = False
        self._passed_tunnel_at: Optional[float] = None
        self._passed_finish = False
        self._finish_since: Optional[float] = None
        self._bridge_started: Optional[float] = None
        self._bridge_near_since: Optional[float] = None
        self._bridge_climb_ended = False
        self._bridge_climb_started_at: Optional[float] = None
        # _bridge_seen_at 记录视觉预告；实际桥面阶段由 BridgeTraversal 确认。
        self._bridge_seen_at: Optional[float] = None
        self._bridge_engaged = False
        self._bridge_engaged_at: Optional[float] = None
        # done 之后的稳定确认：车身平稳够久才交回 WAIT。
        self._bridge_done_at: Optional[float] = None
        self._bridge_settle_since: Optional[float] = None
        # 近端胶带「消失」的连续帧计数，用于判定已到桥头
        self._hazard_near_gone_frames = 0
        self._near_gone_at: Optional[float] = None
        self._cleared_bridge = False
        self._cleared_at: Optional[float] = None
        self._turn_wait_heading: Optional[float] = None
        self._left_line_locked = False
        self._turn_lost_announced = False
        self._turn_search_timed_out = False
        self._turn_fallback_active = False
        # 转弯段找线时的旋转方向：+1 左转、-1 右转。按泡沫在画面哪一侧决定。
        self._turn_search_dir = -1.0
        self._crosswalk_frames = 0
        self._hazard_frames = 0
        self._hazard_near_frames = 0
        self._hazard_gone_frames = 0
        self._finish_frames = 0
        self._stable_since: Optional[float] = None
        self._crouch_started: Optional[float] = None
        self._tape_gone_since: Optional[float] = None
        self._lost_since: Optional[float] = None
        self._current_height = self.stand_height
        self._target_height = self.stand_height
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._started_at = time.monotonic()
        self._height_time = time.monotonic()
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None
        self._video_frames = 0
        self._video_size = (int(self.image_width), int(self.image_height))
        self._hdmi_size = self._detect_hdmi_size()
        self._debug_fullscreen_applied = False

        self.line_detector = DualYellowLineDetector(
            self.search_top_ratio,
            self.search_bottom_ratio,
            self.search_left_ratio,
            self.search_right_ratio,
            self.lane_yellow_h_min,
            self.lane_yellow_h_max,
            self.lane_yellow_s_min,
            self.lane_yellow_v_min,
            self.black_value_max,
            self.black_saturation_max,
            self.min_boundary_lane_contrast,
            self.morphology_kernel,
            self.min_line_width_ratio,
            self.max_line_width_ratio,
            self.expected_line_width_ratio,
            self.width_tolerance_ratio,
            self.min_lane_width_ratio,
            self.max_lane_width_ratio,
            self.expected_lane_width_ratio,
            self.lane_width_tolerance_ratio,
            self.max_center_jump_ratio,
            self.min_valid_row_fraction,
            self.min_confidence,
            self.target_offset_pixels,
        )
        self.scene_detector = TrackSceneDetector(
            self.yellow_h_min,
            self.yellow_h_max,
            self.yellow_s_min,
            self.yellow_v_min,
            self.black_value_max,
            self.black_saturation_max,
            self.crosswalk_top_ratio,
            self.crosswalk_bottom_ratio,
            self.hazard_top_ratio,
            self.hazard_bottom_ratio,
            self.side_margin_ratio,
            self.min_crosswalk_stripes,
            self.min_hazard_transitions,
        )

        if self.show_debug:
            try:
                cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
                if DEBUG_FULLSCREEN:
                    self._apply_debug_fullscreen()
                self._window_ready = True
            except cv2.error as error:
                self.get_logger().error(f"调试窗口创建失败: {error}")

        self._publish_zero("startup")
        self._publish_posture(self._current_height)
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="shiwai_camera", daemon=True
        )
        self._capture_thread.start()
        self._timer = self.create_timer(1.0 / self.control_rate, self._control_timer)
        mode = "运动已启用" if self.enable_motion else "只看画面不发车"
        self._announce(
            f"shiwai_lin_line.py adaptive-speed + exit-recovery + bridge-single + pre-bridge-guard + width-mark-v2 + turn-entry-v1 + wait-heading-v2 版启动，{mode}。过桥状态：approach/crossing/exit_slow/exit_hold/done/fault"
        )
        self._announce(
            f"当前速度 {self.forward_speed:.2f} m/s，"
            f"连续过桥 {self.bridge_climb_speed:.2f} m/s（IMU/odom 判定离桥），"
            f"人行道停 {self.sidewalk_stop_seconds:.0f} 秒"
        )
        if RECORD_DEBUG_VIDEO:
            self._announce(
                f"识别画面会录成视频，保存在 {RECORDINGS_DIR}。"
                "跑完一圈后请停掉程序，文件才会写完"
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "enable_motion": False,
            "camera_backend": "v4l2",
            "camera_id": 0,
            "camera_device": "/dev/video0",
            "image_width": 640,
            "image_height": 480,
            "capture_frame_rate": 30.0,
            "pixel_format": "YUYV",
            "rotation_degrees": 0,
            "mirror_mode": "none",
            "camera_retry_seconds": 1.0,
            "image_timeout_seconds": 0.35,
            "control_rate": 15.0,
            "show_debug": True,
            "debug_window_name": "ShiWai Track (Q/Esc to stop)",
            "search_top_ratio": SEARCH_TOP_RATIO,
            "search_bottom_ratio": SEARCH_BOTTOM_RATIO,
            "search_left_ratio": 0.0,
            "search_right_ratio": 1.0,
            # 巡线泡沫黄边使用视频校准值；与下面的场景黄色阈值独立。
            "lane_yellow_h_min": LANE_YELLOW_H_MIN,
            "lane_yellow_h_max": LANE_YELLOW_H_MAX,
            "lane_yellow_s_min": LANE_YELLOW_S_MIN,
            "lane_yellow_v_min": LANE_YELLOW_V_MIN,
            "yellow_h_min": 12,
            "yellow_h_max": 42,
            "yellow_s_min": 50,
            "yellow_v_min": 70,
            "min_boundary_lane_contrast": 18.0,
            "morphology_kernel": 3,
            "min_line_width_ratio": 0.012,
            "max_line_width_ratio": 0.50,
            "expected_line_width_ratio": 0.08,
            "width_tolerance_ratio": 0.08,
            "min_lane_width_ratio": 0.32,
            "max_lane_width_ratio": 0.82,
            "expected_lane_width_ratio": 0.72,
            "lane_width_tolerance_ratio": 0.30,
            "max_center_jump_ratio": 0.04,
            "min_valid_row_fraction": 0.12,
            "min_confidence": 0.38,
            "target_offset_pixels": 0.0,
            "black_value_max": 95,
            "black_saturation_max": 110,
            "crosswalk_top_ratio": 0.52,
            "crosswalk_bottom_ratio": 0.86,
            "hazard_top_ratio": 0.08,
            "hazard_bottom_ratio": 0.72,
            "side_margin_ratio": 0.03,
            "min_crosswalk_stripes": 4,
            "min_hazard_transitions": 1.8,
            "crosswalk_confirm_frames": 4.0,
            "hazard_confirm_frames": 4.0,
            "hazard_gone_frames": 12.0,
            # 单边桥：近端胶带消失后再等多久才右偏+侧倾；以及兜底超时。
            "bridge_engage_after_gone_seconds": BRIDGE_ENGAGE_AFTER_GONE_SECONDS,
            "bridge_engage_confirm_frames": BRIDGE_ENGAGE_CONFIRM_FRAMES,
            "bridge_engage_timeout_seconds": BRIDGE_ENGAGE_TIMEOUT_SECONDS,
            "sidewalk_stop_seconds": SIDEWALK_STOP_SECONDS,
            "stand_height": STAND_HEIGHT,
            "crouch_height": CROUCH_HEIGHT,
            "height_rate": HEIGHT_RATE,
            "min_crouch_seconds": MIN_CROUCH_SECONDS,
            "tunnel_pass_seconds": TUNNEL_PASS_SECONDS,
            "max_crouch_seconds": MAX_CROUCH_SECONDS,
            "bridge_speed_scale": BRIDGE_SPEED_SCALE,
            "bridge_climb_speed": BRIDGE_CLIMB_SPEED,
            "bridge_climb_seconds": BRIDGE_CLIMB_SECONDS,
            "tunnel_speed_scale": TUNNEL_SPEED_SCALE,
            "lost_line_grace_seconds": 0.50,
            "tunnel_lost_line_grace_seconds": 1.80,
            "forward_speed": FORWARD_SPEED,
            "minimum_forward_speed": MINIMUM_FORWARD_SPEED,
            "linear_direction_sign": 1.0,
            "steering_kp": 1.00,
            "steering_kd": 0.025,
            "steering_sign": -1.0,
            "max_angular_speed": MAX_ANGULAR_SPEED,
            "max_linear_acceleration": 0.25,
            "max_angular_acceleration": 1.00,
            "max_error_derivative": 3.0,
            "error_filter_alpha": 0.45,
            "curve_slowdown": CURVE_SLOWDOWN,
            "status_log_period": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        for parameter in self._parameters.values():
            setattr(self, parameter.name, self.get_parameter(parameter.name).value)
        integer_names = (
            "camera_id",
            "image_width",
            "image_height",
            "rotation_degrees",
            "lane_yellow_h_min",
            "lane_yellow_h_max",
            "lane_yellow_s_min",
            "lane_yellow_v_min",
            "yellow_h_min",
            "yellow_h_max",
            "yellow_s_min",
            "yellow_v_min",
            "morphology_kernel",
            "black_value_max",
            "black_saturation_max",
            "min_crosswalk_stripes",
        )
        bool_names = ("enable_motion", "show_debug")
        string_names = (
            "camera_backend",
            "camera_device",
            "pixel_format",
            "mirror_mode",
            "debug_window_name",
        )
        for name in integer_names:
            setattr(self, name, int(getattr(self, name)))
        for name in bool_names:
            setattr(self, name, bool(getattr(self, name)))
        for name in string_names:
            setattr(self, name, str(getattr(self, name)))
        self.camera_backend = self.camera_backend.lower()
        self.pixel_format = self.pixel_format.upper()
        self.mirror_mode = self.mirror_mode.lower()
        for name in self._parameters:
            if name not in integer_names + bool_names + string_names:
                setattr(self, name, float(getattr(self, name)))

    def _apply_field_tuning(self) -> None:
        """文件开头的现场参数覆盖 yaml，改脚本顶部即可生效。"""
        self.forward_speed = float(FORWARD_SPEED)
        self.minimum_forward_speed = float(MINIMUM_FORWARD_SPEED)
        self.max_angular_speed = float(MAX_ANGULAR_SPEED)
        self.steering_kp = float(STEERING_KP)
        self.curve_slowdown = float(CURVE_SLOWDOWN)
        self.bridge_speed_scale = float(BRIDGE_SPEED_SCALE)
        self.bridge_climb_speed = float(BRIDGE_CLIMB_SPEED)
        self.bridge_climb_seconds = float(BRIDGE_CLIMB_SECONDS)
        self.tunnel_speed_scale = float(TUNNEL_SPEED_SCALE)
        self.sidewalk_stop_seconds = float(SIDEWALK_STOP_SECONDS)
        self.stand_height = float(STAND_HEIGHT)
        self.crouch_height = float(CROUCH_HEIGHT)
        self.height_rate = float(HEIGHT_RATE)
        self.min_crouch_seconds = float(MIN_CROUCH_SECONDS)
        self.tunnel_pass_seconds = float(TUNNEL_PASS_SECONDS)
        self.max_crouch_seconds = float(MAX_CROUCH_SECONDS)
        self.target_offset_pixels = float(TARGET_OFFSET_PIXELS)
        self.max_lane_width_ratio = 0.82
        self.search_top_ratio = float(SEARCH_TOP_RATIO)
        self.search_bottom_ratio = float(SEARCH_BOTTOM_RATIO)
        self.crosswalk_top_ratio = float(CROSSWALK_TOP_RATIO)
        self.crosswalk_bottom_ratio = float(CROSSWALK_BOTTOM_RATIO)
        self.crosswalk_confirm_frames = float(CROSSWALK_CONFIRM_FRAMES)
        self.min_crosswalk_stripes = float(MIN_CROSSWALK_STRIPES)
        self.crosswalk_max_yellow_frac = float(CROSSWALK_MAX_YELLOW_FRAC)

    def _on_bridge_section(self) -> bool:
        # 用 _bridge_engaged 而不是 _passed_bridge：
        # 桥区姿态目标沿用现有参数；状态判定由 IMU/odom 负责。
        return (
            self._bridge_engaged
            and not self._cleared_bridge
            and not self._passed_sidewalk
        )

    def _update_bridge_target(self) -> None:
        """上桥往右偏；下桥后只跟左侧海绵过大弯。"""
        if self._on_bridge_section():
            offset_cm = BRIDGE_OFFSET_CM
        elif self._cleared_bridge and not self._passed_sidewalk:
            offset_cm = TURN_OFFSET_CM
        else:
            offset_cm = 0.0
        offset = (offset_cm / TRACK_WIDTH_CM) * (
            self.image_width * self.expected_lane_width_ratio
        )
        # 每帧都会重算，所以静态偏置必须加在这里，不能只写在 _apply_field_tuning。
        self.line_detector.target_offset_pixels = float(
            offset + TARGET_OFFSET_PIXELS
        )
        # 冲桥阶段（crossing / exit_slow / exit_hold）必须保持原有算法，
        # 这一段不参与最长白列检测；进桥前和下桥后的大弯才用它。
        self.line_detector.column_scan = (
            COLUMN_SCAN_ENABLED and not self._bridge.crossing
        )
        self.line_detector.expected_turn_direction = (
            1 if self._cleared_bridge and not self._passed_sidewalk else 0
        )
        self.line_detector.prefer_left_line = not self._passed_sidewalk
        self.line_detector.pre_bridge_guard = (
            not self._cleared_bridge and not self._passed_sidewalk
            and not self._bridge.crossing
        )
        self.line_detector.width_profile = (
            "crouched" if self._mode == self.MODE_CROUCH else "standing"
        )
        if self._mode == self.MODE_CROUCH:
            self.line_detector.single_left_half_ratio = TUNNEL_SINGLE_LEFT_HALF_RATIO
            self.line_detector.single_right_half_ratio = TUNNEL_SINGLE_RIGHT_HALF_RATIO
            self.line_detector.near_lock_ratio = self.line_detector.base_near_lock_ratio
            self.line_detector.search_bottom_ratio = float(SEARCH_BOTTOM_RATIO)
            self.line_detector.min_valid_row_fraction = (
                self.line_detector.base_min_valid_row_fraction
            )
            self.line_detector.min_single_coverage = (
                self.line_detector.base_min_single_coverage
            )
            self.line_detector.force_left_line = False
        elif self._cleared_bridge and not self._passed_sidewalk:
            self.line_detector.single_left_half_ratio = TURN_SINGLE_LEFT_HALF_RATIO
            self.line_detector.single_right_half_ratio = TURN_SINGLE_RIGHT_HALF_RATIO
            self.line_detector.near_lock_ratio = TURN_NEAR_LOCK_RATIO
            # 转弯段把 ROI 底部放宽，否则近处泡沫一贴底就整幅消失。
            self.line_detector.search_bottom_ratio = TURN_SEARCH_BOTTOM_RATIO
            self.line_detector.min_valid_row_fraction = 0.06
            self.line_detector.min_single_coverage = 0.10
            self.line_detector.force_left_line = True
        else:
            self.line_detector.single_left_half_ratio = SINGLE_LEFT_HALF_RATIO
            self.line_detector.single_right_half_ratio = SINGLE_RIGHT_HALF_RATIO
            self.line_detector.near_lock_ratio = self.line_detector.base_near_lock_ratio
            self.line_detector.search_bottom_ratio = float(SEARCH_BOTTOM_RATIO)
            self.line_detector.min_valid_row_fraction = (
                self.line_detector.base_min_valid_row_fraction
            )
            self.line_detector.min_single_coverage = (
                self.line_detector.base_min_single_coverage
            )
            self.line_detector.force_left_line = False

    def _validate_parameters(self) -> None:
        if self.camera_backend not in ("auto", "v4l2"):
            raise ValueError("camera_backend 只能是 auto 或 v4l2")
        if self.camera_id < 0 or self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("摄像头编号和图像尺寸无效")
        if self.capture_frame_rate <= 0.0:
            raise ValueError("capture_frame_rate 必须大于 0")
        if self.rotation_degrees not in self.VALID_ROTATIONS:
            raise ValueError("rotation_degrees 必须是 0/90/180/270")
        if self.mirror_mode not in self.VALID_MIRRORS:
            raise ValueError("mirror_mode 无效")
        if len(self.pixel_format) != 4:
            raise ValueError("pixel_format 必须是四字符格式")
        if not 10.0 <= self.control_rate <= 20.0:
            raise ValueError("control_rate 必须在 10~20 Hz")
        if not 0 <= self.lane_yellow_h_min < self.lane_yellow_h_max <= 179:
            raise ValueError("巡线黄色色调范围无效")
        if not (
            0 <= self.lane_yellow_s_min <= 255
            and 0 <= self.lane_yellow_v_min <= 255
        ):
            raise ValueError("巡线黄色饱和度或亮度阈值无效")
        if not 0 <= self.yellow_h_min < self.yellow_h_max <= 180:
            raise ValueError("黄色色调范围无效")
        if not 0 <= self.yellow_s_min <= 255 or not 0 <= self.yellow_v_min <= 255:
            raise ValueError("黄色饱和度或亮度阈值无效")
        if not 0.0 < self.min_boundary_lane_contrast <= 255.0:
            raise ValueError("min_boundary_lane_contrast 必须在 (0,255]")
        ratios = (
            "min_line_width_ratio",
            "max_line_width_ratio",
            "expected_line_width_ratio",
            "width_tolerance_ratio",
            "min_lane_width_ratio",
            "max_lane_width_ratio",
            "expected_lane_width_ratio",
            "lane_width_tolerance_ratio",
            "max_center_jump_ratio",
            "min_valid_row_fraction",
            "min_confidence",
            "error_filter_alpha",
            "curve_slowdown",
            "side_margin_ratio",
            "crosswalk_top_ratio",
            "crosswalk_bottom_ratio",
            "hazard_top_ratio",
            "hazard_bottom_ratio",
        )
        for name in ratios:
            if not 0.0 < getattr(self, name) <= 1.0:
                raise ValueError(f"{name} 必须在 (0,1]")
        if not 0.0 <= self.search_top_ratio < self.search_bottom_ratio <= 1.0:
            raise ValueError("搜索区域上下比例无效")
        if not 0.0 <= self.search_left_ratio < self.search_right_ratio <= 1.0:
            raise ValueError("搜索区域左右比例无效")
        if not self.crosswalk_top_ratio < self.crosswalk_bottom_ratio:
            raise ValueError("人行道识别区域无效")
        if not self.hazard_top_ratio < self.hazard_bottom_ratio:
            raise ValueError("警戒胶带识别区域无效")
        if not self.min_line_width_ratio < self.max_line_width_ratio:
            raise ValueError("黄线宽度范围无效")
        if not (
            self.min_line_width_ratio
            <= self.expected_line_width_ratio
            <= self.max_line_width_ratio
        ):
            raise ValueError("expected_line_width_ratio 必须在线宽范围内")
        if not self.min_lane_width_ratio < self.max_lane_width_ratio:
            raise ValueError("双线间距范围无效")
        if not (
            self.min_lane_width_ratio
            <= self.expected_lane_width_ratio
            <= self.max_lane_width_ratio
        ):
            raise ValueError("expected_lane_width_ratio 必须在双线间距范围内")
        if self.linear_direction_sign not in (-1.0, 1.0):
            raise ValueError("linear_direction_sign 只能是 -1 或 1")
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError("steering_sign 只能是 -1 或 1")
        if not 0.0 <= self.minimum_forward_speed <= self.forward_speed:
            raise ValueError("前进速度范围无效")
        if not 0.14 <= self.crouch_height < self.stand_height <= 0.36:
            raise ValueError("站立高度必须高于下蹲高度，且都在 0.14~0.36 m")
        if self.height_rate <= 0.0:
            raise ValueError("height_rate 必须大于 0")
        if self.sidewalk_stop_seconds <= 0.0:
            raise ValueError("sidewalk_stop_seconds 必须大于 0")
        if not 0.0 < self.min_crouch_seconds <= self.max_crouch_seconds:
            raise ValueError("下蹲时间范围无效")
        if self.bridge_climb_speed <= 0.0:
            raise ValueError("bridge_climb_speed 必须大于 0")
        if self.bridge_climb_seconds <= 0.0:
            raise ValueError("bridge_climb_seconds 必须大于 0")
        if self.bridge_engage_after_gone_seconds < 0.0:
            raise ValueError("bridge_engage_after_gone_seconds 不能为负")
        if self.bridge_engage_confirm_frames < 1.0:
            raise ValueError("bridge_engage_confirm_frames 至少 1 帧")
        if self.bridge_engage_timeout_seconds <= self.bridge_engage_after_gone_seconds:
            raise ValueError(
                "bridge_engage_timeout_seconds 必须大于 bridge_engage_after_gone_seconds"
            )
        if self.tunnel_pass_seconds <= 0.0:
            raise ValueError("tunnel_pass_seconds 必须大于 0")
        if (
            self.max_angular_speed <= 0.0
            or self.max_linear_acceleration <= 0.0
            or self.max_angular_acceleration <= 0.0
            or self.max_error_derivative <= 0.0
            or self.status_log_period <= 0.0
        ):
            raise ValueError("速度、加速度、误差变化率和日志周期必须大于 0")
        if self.steering_kp < 0.0 or self.steering_kd < 0.0:
            raise ValueError("转向增益不能为负")
        if self.camera_retry_seconds <= 0.0 or self.image_timeout_seconds <= 0.0:
            raise ValueError("摄像头重试和图像超时必须大于 0")

    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        if self.camera_backend == "v4l2":
            capture = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(self.camera_id)
        if not capture.isOpened():
            capture.release()
            return None
        if self.camera_backend == "v4l2":
            capture.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format)
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        capture.set(cv2.CAP_PROP_FPS, self.capture_frame_rate)
        self.get_logger().info(
            f"摄像头: {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
            f"{capture.get(cv2.CAP_PROP_FPS):.1f} fps"
        )
        return capture

    def _capture_loop(self) -> None:
        while not self._stop_capture.is_set():
            with self._capture_lock:
                capture = self._capture
            if capture is None or not capture.isOpened():
                capture = self._open_camera()
                with self._capture_lock:
                    self._capture = capture
                if capture is None:
                    with self._frame_lock:
                        self._camera_ok = False
                        self._frame = None
                    self._publish_zero("camera_open_failed")
                    self._stop_capture.wait(self.camera_retry_seconds)
                    continue
            ok, frame = capture.read()
            if not ok or frame is None:
                with self._frame_lock:
                    self._camera_ok = False
                    self._frame = None
                self._publish_zero("camera_read_failed")
                capture.release()
                with self._capture_lock:
                    self._capture = None
                self._stop_capture.wait(self.camera_retry_seconds)
                continue
            frame = self._transform(frame)
            with self._frame_lock:
                self._frame = frame
                self._frame_time = time.monotonic()
                self._sequence += 1
                self._camera_ok = True

    def _transform(self, frame: np.ndarray) -> np.ndarray:
        if self.rotation_degrees == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation_degrees == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation_degrees == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.mirror_mode == "horizontal":
            frame = cv2.flip(frame, 1)
        elif self.mirror_mode == "vertical":
            frame = cv2.flip(frame, 0)
        elif self.mirror_mode == "both":
            frame = cv2.flip(frame, -1)
        return frame

    def _publish(self, forward_speed: float, angular_speed: float) -> None:
        message = Twist()
        message.linear.x = self.linear_direction_sign * float(forward_speed)
        message.angular.z = float(angular_speed)
        self.cmd_vel_pub.publish(message)

    def _publish_posture(self, height: float) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["joint_height", "joint_roll", "joint_pitching"]
        message.position = [
            float(np.clip(height, 0.14, 0.36)),
            float(np.clip(self._current_roll, -15.0, 15.0)),
            float(np.clip(self._current_pitch, -35.0, 35.0)),
        ]
        self.posture_pub.publish(message)

    def _bridge_roll_target(self) -> float:
        if not self._on_bridge_section():
            return 0.0
        if time.monotonic() - self._started_at < BRIDGE_ROLL_DELAY_SECONDS:
            return 0.0
        return float(BRIDGE_ROLL_DEG)

    def _update_height(self, now: float) -> None:
        dt = max(0.0, min(now - self._height_time, 2.0 / self.control_rate))
        self._height_time = now
        delta = self.height_rate * dt
        self._current_height = float(
            np.clip(
                self._target_height,
                self._current_height - delta,
                self._current_height + delta,
            )
        )
        self._current_roll = self._approach(
            self._current_roll,
            self._bridge_roll_target(),
            ROLL_RATE_DEG * dt,
        )
        # 低头是全程目标，不随任何赛道阶段变化。
        self._current_pitch = self._approach(
            self._current_pitch,
            float(PITCH_DEG),
            PITCH_RATE_DEG * dt,
        )
        self._publish_posture(self._current_height)

    def _publish_zero(self, _reason: str) -> None:
        if _reason != "motion_disabled":
            self._reset_post_bridge_speed("stop")
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass
        self._last_command = (0.0, 0.0)
        self._last_command_valid = False
        self._last_motion_time = time.monotonic()
        self._have_error = False

    @staticmethod
    def _approach(current: float, target: float, delta: float) -> float:
        return float(np.clip(target, current - delta, current + delta))

    def _rate_limit(self, linear: float, angular: float, now: float):
        dt = max(0.0, min(now - self._last_motion_time, 2.0 / self.control_rate))
        acceleration = self.max_linear_acceleration
        if self._bridge.boost and linear > self._last_command[0]:
            acceleration = BRIDGE_ACCELERATION
        lin_step = acceleration * dt
        if linear < self._last_command[0]:
            lin_step *= 3.0
        linear = self._approach(self._last_command[0], linear, lin_step)
        angular = self._approach(
            self._last_command[1], angular, self.max_angular_acceleration * dt
        )
        self._last_motion_time = now
        return linear, angular

    def _confirm(self, frames_attr: str, seen: bool, needed: float) -> bool:
        current = int(getattr(self, frames_attr))
        current = current + 1 if seen else 0
        setattr(self, frames_attr, current)
        return current >= int(needed)

    def _holding_stop(self) -> bool:
        return self._mode in (self.MODE_STOP, self.MODE_FINISH)

    def _update_mission(self, scene: SceneObservation, now: float) -> None:
        if self._holding_stop():
            return

        hazard_confirmed = self._confirm(
            "_hazard_frames", scene.hazard, self.hazard_confirm_frames
        )
        hazard_gone = self._confirm(
            "_hazard_gone_frames",
            (not scene.hazard) and self._hazard_frames == 0,
            self.hazard_gone_frames,
        )
        crosswalk_confirmed = self._confirm(
            "_crosswalk_frames",
            scene.crosswalk and not self._passed_sidewalk,
            self.crosswalk_confirm_frames,
        )

        near_confirmed = self._confirm(
            "_hazard_near_frames",
            scene.hazard_near and not self._passed_sidewalk,
            max(2, int(self.hazard_confirm_frames) - 2),
        )
        if (hazard_confirmed or near_confirmed) and not self._passed_sidewalk:
            if self._bridge.state == "idle":
                self._passed_bridge = True
                self._bridge_seen_at = self._bridge_started = now
                self._bridge.arm(now)
            if near_confirmed:
                self._bridge.near = True
        self._sync_bridge(now)
        bridge_pending = self._bridge.state not in ("idle", "done")

        if crosswalk_confirmed and not self._passed_sidewalk and not bridge_pending:
            self._mode = self.MODE_STOP
            self._stable_since = None
            self._have_error = False
            self._target_height = self.stand_height
            self._mark_bridge_climb_ended("到达人行道，单边桥加速结束")
            self._announce(
                f"识别到人行道，开始减速停车，停稳后再计时 "
                f"{self.sidewalk_stop_seconds:.0f} 秒"
            )
            return

        if (
            hazard_confirmed
            and self._passed_sidewalk
            and not self._passed_tunnel
            and self._mode != self.MODE_CROUCH
        ):
            self._mode = self.MODE_CROUCH
            self._crouch_started = now
            self._tape_gone_since = None
            self._target_height = self.crouch_height
            self._announce(
                f"开始过隧道：下蹲到 {self.crouch_height:.3f} m，减速通过"
            )
            return

        if self._mode == self.MODE_CROUCH and self._crouch_started is not None:
            crouched = now - self._crouch_started
            if hazard_gone:
                if self._tape_gone_since is None:
                    self._tape_gone_since = now
                    self._announce("隧道入口已过，继续蹲着往前走")
            else:
                self._tape_gone_since = None
            passed_interior = (
                self._tape_gone_since is not None
                and now - self._tape_gone_since >= self.tunnel_pass_seconds
                and crouched >= self.min_crouch_seconds
            )
            if crouched >= self.max_crouch_seconds or passed_interior:
                self._mode = self.MODE_FOLLOW
                self._passed_tunnel = True
                self._passed_tunnel_at = now
                self._tape_gone_since = None
                self._target_height = self.stand_height
                self._announce("已出隧道，恢复站立，继续巡线")

        tunnel_ready_for_finish = (
            (self._passed_tunnel
             and self._passed_tunnel_at is not None
             and (now - self._passed_tunnel_at) >= FINISH_AFTER_TUNNEL_SECONDS)
            # 终点字可能就在隧道末段：下蹲够久就允许认，不再等 _passed_tunnel。
            or (self._mode == self.MODE_CROUCH
                and self._crouch_started is not None
                and now - self._crouch_started >= FINISH_IN_TUNNEL_MIN_SECONDS)
        )
        finish_confirmed = self._confirm(
            "_finish_frames",
            scene.finish
            and tunnel_ready_for_finish
            and not self._passed_finish,
            FINISH_CONFIRM_FRAMES,
        )
        if finish_confirmed:
            self._mode = self.MODE_FINISH
            self._passed_finish = True
            self._finish_since = now
            self._have_error = False
            self._target_height = self.stand_height
            self._announce(
                f"到达终点，再往前驶入 {FINISH_DRIVE_IN_SECONDS:.1f} 秒后停车"
            )
            return

    def _announce(self, text: str) -> None:
        line = f">>> [赛道] {text}"
        print(f"\n{line}\n", flush=True)
        self.get_logger().info(line)

    def _mark_bridge_climb_ended(self, text: str) -> None:
        if self._bridge_started is None or self._bridge_climb_ended:
            return
        self._bridge_climb_ended = True
        self._announce(text)

    def _imu_callback(self, msg: Imu) -> None:
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        age = (self.get_clock().now().nanoseconds - (stamp[0]*10**9+stamp[1])) / 1e9
        if not 0.0 <= age <= BRIDGE_SENSOR_TIMEOUT:
            return
        if stamp == self._last_imu_stamp:
            return
        self._last_imu_stamp = stamp
        q = msg.orientation
        # 当前 wl_base_node 将 q0,q1,q2,q3 填进 x,y,z,w；只在本文件兼容。
        quaternion = ((q.x, q.y, q.z, q.w) if BRIDGE_IMU_LEGACY_ORDER
                      else (q.w, q.x, q.y, q.z))
        g = msg.angular_velocity
        self._bridge.imu(quaternion, (g.x, g.y, g.z), time.monotonic())

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        age = (self.get_clock().now().nanoseconds - (stamp[0]*10**9+stamp[1])) / 1e9
        if not 0.0 <= age <= BRIDGE_SENSOR_TIMEOUT:
            return
        if stamp == self._last_odom_stamp:
            return
        self._last_odom_stamp = stamp
        self._bridge.odom(msg.twist.twist.linear.x, time.monotonic())

    def _sync_bridge(self, now: float) -> None:
        b = self._bridge
        b.tick(now)
        if b.boost and not self._bridge_engaged:
            self._bridge_engaged = True
            self._bridge_engaged_at = now
            self._bridge_climb_started_at = now
        if b.state != self._bridge_announced_state:
            self._bridge_announced_state = b.state
            messages = {
                "approach": "视觉预告单边桥：连续巡线接近，近端提速，等待横滚确认",
                "crossing": "IMU 确认桥上倾斜：连续通过，等待回平，不按两秒结束",
                "exit_slow": "检测到回平趋势：锁定低速离桥确认，不再恢复冲桥；确认后交回弯道循迹",
                "exit_hold": "离桥前进预算已用完：停止直行并继续回平确认，成功后自动恢复循迹",
                "done": "姿态与里程共同确认离桥：交回桥后循迹",
                "fault": f"过桥异常停车（未判定过桥）：{b.reason}；需检查后重新启动节点",
            }
            if b.state in messages:
                self._announce(messages[b.state])
        if b.state == "done" and not self._cleared_bridge:
            if self._turn_wait_heading is None and math.isfinite(b.yaw):
                # 记录完成离桥时的航向：稳定确认期间用它保航向。
                self._turn_wait_heading = float(b.yaw)
            if not self._bridge_settle_ready(now):
                return
            self._cleared_bridge = True
            self._cleared_at = now
            # 车身已经稳了，用稳定后的航向作为 WAIT 参考，不复用上桥航向。
            if math.isfinite(b.yaw):
                self._turn_wait_heading = float(b.yaw)
            self._bridge_climb_ended = True
            self._left_line_locked = False
            self._have_error = False
            self._lost_since = None

    def _bridge_settle_ready(self, now: float) -> bool:
        """done 之后确认车身真的稳了；超过 BRIDGE_SETTLE_TIMEOUT 也放行。"""
        b = self._bridge
        if self._bridge_done_at is None:
            self._bridge_done_at = now
        if now - self._bridge_done_at >= BRIDGE_SETTLE_TIMEOUT:
            return True
        if b.baseline is None or b.imu_at is None:
            return False
        if not 0.0 <= now - b.imu_at <= BRIDGE_SENSOR_TIMEOUT:
            self._bridge_settle_since = None
            return False
        dr = abs(b.angle_delta(b.roll, b.baseline[0]))
        dp = abs(b.angle_delta(b.pitch, b.baseline[1]))
        calm = (dr <= BRIDGE_SETTLE_ROLL_DEG and dp <= BRIDGE_SETTLE_PITCH_DEG
                and abs(b.roll_rate) <= BRIDGE_SETTLE_RATE_DEG)
        if not calm:
            self._bridge_settle_since = None
            return False
        if self._bridge_settle_since is None:
            self._bridge_settle_since = now
        return now - self._bridge_settle_since >= BRIDGE_SETTLE_SECONDS

    def _bridge_settle_command(self, result: DetectionResult, now: float) -> None:
        """过桥后低速直行等车身稳住；稳住后才交回 WAIT。"""
        b = self._bridge
        self._have_error = False
        heading = self._turn_wait_heading
        if heading is None or not math.isfinite(heading):
            heading = self._turn_wait_heading = (
                float(b.yaw) if math.isfinite(b.yaw) else 0.0)
        imu_ok = (b.imu_at is not None
                  and 0.0 <= now - b.imu_at <= BRIDGE_SENSOR_TIMEOUT
                  and math.isfinite(b.yaw))
        if imu_ok:
            error = math.radians(b.angle_delta(heading, b.yaw))
            angular = float(np.clip(TURN_WAIT_HEADING_KP*error,
                                    -TURN_ENTRY_MAX_W, TURN_ENTRY_MAX_W))
            linear = float(BRIDGE_SETTLE_SPEED)
        else:
            # 没有 IMU 就不动，避免盲走。
            linear = angular = 0.0
        if self.enable_motion:
            command = self._rate_limit(linear, angular, now)
            self._publish(*command)
            self._last_command, self._last_command_valid = command, True
        else:
            command = (linear, angular)
            self._publish_zero("motion_disabled")
        self._update_height(now)
        dr = 0.0 if b.baseline is None else b.angle_delta(b.roll, b.baseline[0])
        dp = 0.0 if b.baseline is None else b.angle_delta(b.pitch, b.baseline[1])
        self._show(result, *command, self.enable_motion,
                   extra=f"BRIDGE SETTLE droll={dr:+.1f} dpitch={dp:+.1f}")

    def _bridge_steering(self, result: DetectionResult) -> Tuple[float, str]:
        """复用当前检测：双边沿用原修正，可靠单边主导纠偏，其余保持航向。"""
        b = self._bridge
        yaw_error = math.radians(b.angle_delta(b.heading, b.yaw))
        heading = BRIDGE_HEADING_KP * yaw_error
        angular, guide = heading, "IMU"
        if (result.valid and math.isfinite(result.error)
                and math.isfinite(result.confidence)):
            left, right = result.left_line_rows, result.right_line_rows
            strong, other = max(left, right), min(left, right)
            dual = (result.reason == "tracking" and result.confidence >= 0.85
                    and other >= 40)
            single = (
                result.reason in ("left_line", "single_line", "tracking")
                and result.confidence >= BRIDGE_SINGLE_MIN_CONFIDENCE
                and strong >= BRIDGE_SINGLE_MIN_ROWS
                and other <= BRIDGE_SINGLE_MAX_OTHER_RATIO * strong
            )
            if dual or single:
                visual = self.steering_sign * self.steering_kp * result.error
                visual = float(np.clip(visual, -BRIDGE_VISUAL_MAX_W, BRIDGE_VISUAL_MAX_W))
                if dual:
                    angular, guide = heading + visual, "LR"
                else:
                    # 单边纠偏不能被入桥时锁住的旧航向完全抵消或反转。
                    # 误差归零时辅助项也归零；不把当前 yaw 写回桥状态机基准。
                    assist_limit = BRIDGE_SINGLE_IMU_RATIO * abs(visual)
                    assist = max(-assist_limit, min(heading, assist_limit))
                    angular, guide = visual + assist, "L" if left > right else "R"
        return float(np.clip(angular, -BRIDGE_MAX_W, BRIDGE_MAX_W)), guide

    def _bridge_command(self, result: DetectionResult, now: float) -> None:
        """桥上允许可靠单边持续纠偏，保持原速度、限幅与停车逻辑。"""
        b = self._bridge
        if b.state == "exit_hold":
            # 滚行确认回平：不停车，也不累计桥上丢线故障。
            #
            # ⚠ 以前这里直接 _publish_zero：2129/211948 实测 4.533s→4.600s
            # 一帧从 0.24 m/s 砸到 0.00（等效 −3.6 m/s²），接着又从 0 以
            # max_linear_acceleration(0.25) 爬回 0.16 要 0.64s —— 下桥顿挫全在这。
            # 现在改为低速滚行；回平判到后照旧进 done，3s 仍不回平仍会 fault。
            self._have_error = False
            creep = float(BRIDGE_EXIT_HOLD_SPEED)
            if self.enable_motion:
                command = self._rate_limit(creep, 0.0, now)
                self._publish(*command)
                self._last_command, self._last_command_valid = command, True
            else:
                command = (creep, 0.0)
                self._publish_zero("motion_disabled")
            self._update_height(now)
            self._show(result, *command, self.enable_motion,
                       extra="EXIT ROLL / LEVEL CHECK")
            return
        if result.valid:
            self._lost_since = None
        elif self._lost_since is None:
            self._lost_since = now
        lost_for = 0.0 if self._lost_since is None else now-self._lost_since
        if b.state == "crossing" and lost_for >= BRIDGE_LOST_LINE_TIMEOUT:
            b.fail("bridge lane lost too long")
            self._sync_bridge(now)
            self._publish_zero("bridge_lane_lost")
            self._show(result, 0.0, 0.0, False, extra="BRIDGE FAULT")
            return
        linear = self.bridge_climb_speed
        if b.state == "exit_slow":
            linear = min(linear, BRIDGE_EXIT_SPEED)
        if lost_for > BRIDGE_LOST_LINE_HOLD:
            linear = min(linear, BRIDGE_LOST_LINE_SPEED)
        angular, guide = self._bridge_steering(result)
        if self.enable_motion:
            command = self._rate_limit(linear, angular, now)
            self._publish(*command)
            self._last_command, self._last_command_valid = command, True
        else:
            command = (linear, angular)
            self._publish_zero("motion_disabled")
        self._update_height(now)
        self._show(result, *command, self.enable_motion, extra=f"guide={guide}")

    def _speed_scale(self, now: float) -> float:
        if self._mode == self.MODE_CROUCH:
            return self.tunnel_speed_scale
        if self._bridge.boost:
            return self.bridge_climb_speed / max(self.forward_speed, 1e-3)
        # 桥后速度由 _post_bridge_speed 按当前边线/误差/转向需求决定。
        return 1.0

    def _control_timer(self) -> None:
        try:
            self._control_cycle()
        except Exception as error:
            self.get_logger().error(f"控制异常，立即停车: {error}")
            self._publish_zero("control_exception")
            try:
                self._current_roll = 0.0
                self._current_pitch = 0.0
                self._publish_posture(self.stand_height)
            except Exception:
                pass

    def _control_cycle(self) -> None:
        now = time.monotonic()
        self._sync_bridge(now)
        if self._bridge.state == "fault":
            self._publish_zero("bridge_fault")
        with self._frame_lock:
            ok = self._camera_ok
            frame_time = self._frame_time
            sequence = self._sequence
            frame = self._frame.copy() if self._frame is not None else None
        if not ok or frame is None:
            if self._holding_stop():
                self._publish(0.0, 0.0)
            else:
                self._publish_zero("camera_unavailable")
            self._update_height(now)
            self._log(now, "摄像头不可用，停车")
            return
        if now - frame_time > self.image_timeout_seconds:
            if self._holding_stop():
                self._publish(0.0, 0.0)
            else:
                self._publish_zero("image_timeout")
            self._update_height(now)
            self._log(now, "画面超时，停车")
            return
        if sequence == self._last_sequence:
            if self._holding_stop():
                self._publish(0.0, 0.0)
            elif self.enable_motion and self._last_command_valid:
                self._publish(*self._last_command)
            else:
                self._publish_zero("no_new_frame")
            self._update_height(now)
            return
        self._last_sequence = sequence

        self._update_bridge_target()
        result = self._detect_lane(frame)
        scene = self.scene_detector.detect(
            frame, result.debug_frame,
            prefer_finish=(self._passed_tunnel or self._mode == self.MODE_CROUCH),
        )
        self._update_mission(scene, now)
        self._log_status(now, result, scene)

        if self._mode == self.MODE_STOP:
            self._hold_sidewalk_stop(result, now)
            return
        if self._mode == self.MODE_FINISH:
            self._hold_finish_stop(result, now)
            return
        if self._bridge.state == "fault":
            self._publish_zero("bridge_fault")
            self._update_height(now)
            self._show(result, 0.0, 0.0, False, extra="BRIDGE FAULT")
            return
        if self._bridge.crossing:
            self._bridge_command(result, now)
            return
        if self._bridge.state == "done" and not self._cleared_bridge:
            # 过桥后先低速直行等车身稳住，再进 WAIT。
            self._bridge_settle_command(result, now)
            return

        if self._should_search_after_bridge(result, now):
            self._handle_lost_line(result, now, searching=True)
            return
        if not result.valid:
            self._handle_lost_line(result, now)
            return
        self._lost_since = None

        command = self._follow_command(result, now)
        self._update_height(now)
        self._show(result, *command, self.enable_motion, scene)

    def _reset_post_bridge_speed(self, mode: str = "wait") -> None:
        self._post_bridge_samples.clear()
        self._post_bridge_last_sample = None
        self._post_bridge_speed_mode = mode
        self._post_bridge_speed_target = 0.0

    def _post_bridge_speed(
        self, result: DetectionResult, angular: float, now: float
    ) -> float:
        """稳定直线走正常速度；转弯、低质量边线、恢复初期沿用保守速度。

        速度目标仍经过 _rate_limit，既不会瞬间加速，也不会一帧跳变刹停。
        使用原始误差的时间窗口，避免滤波后的误差掩盖抖动或突然偏离。
        """
        previous_time = self._post_bridge_last_sample
        if previous_time is not None and not 0.0 < now-previous_time <= POST_BRIDGE_MAX_SAMPLE_GAP:
            self._reset_post_bridge_speed()
        self._post_bridge_last_sample = now
        quality = (
            result.valid and result.reason == "left_line" and self._left_line_locked
            and result.confidence >= POST_BRIDGE_MIN_CONFIDENCE
            and (result.left_line_rows >= POST_BRIDGE_MIN_LEFT_ROWS
                 or (result.curve_mode and result.boundary_side in ("left", "both")
                     and result.measured_points >= POST_BRIDGE_MIN_LEFT_ROWS))
            and math.isfinite(result.error) and math.isfinite(angular)
        )
        straight = (
            quality and result.turn_strength < 0.20
            and abs(result.error) <= POST_BRIDGE_SMALL_ERROR
            and abs(angular) <= POST_BRIDGE_SMALL_ANGULAR
        )
        if straight:
            self._post_bridge_samples.append((now, result.error))
            while self._post_bridge_samples and now-self._post_bridge_samples[0][0] > POST_BRIDGE_HISTORY_SECONDS:
                self._post_bridge_samples.popleft()
        else:
            self._post_bridge_samples.clear()
        samples = self._post_bridge_samples
        stable = (
            straight and len(samples) >= POST_BRIDGE_MIN_SAMPLES
            and now-samples[0][0] >= POST_BRIDGE_STABLE_SECONDS
            and max(e for _, e in samples)-min(e for _, e in samples) <= POST_BRIDGE_MAX_ERROR_SPAN
        )
        if stable:
            target = self.forward_speed
            self._post_bridge_speed_mode = "cruise"
        else:
            turn_ratio = min(1.0, abs(angular) / self.max_angular_speed)
            target = max(
                TURN_MIN_SPEED,
                self.forward_speed * TURN_SPEED_SCALE * (1.0-self.curve_slowdown*turn_ratio),
            )
            self._post_bridge_speed_mode = (
                "weak-line" if not quality else ("settling" if straight else "turn")
            )
        # 出弯后压到低速，避免越过斑马线。
        if result.turn_exit_ready and target > TURN_EXIT_SPEED:
            target = TURN_EXIT_SPEED
            self._post_bridge_speed_mode = "exit-crawl"
        self._post_bridge_speed_target = target
        return target

    def _turn_wait_command(self, now: float) -> Tuple[float, float]:
        """WAIT 不循线：保持离桥时航向，速度使用正常巡线值。"""
        b = self._bridge
        # 不能把 WAIT 画面里的黄线误差留到入弯后的 PID 里。
        self._have_error = False
        imu_at = b.imu_at
        if (imu_at is None or not 0.0 <= now-imu_at <= BRIDGE_SENSOR_TIMEOUT
                or not math.isfinite(b.yaw)):
            self._publish_zero("turn_wait_imu_stale")
            self._reset_post_bridge_speed("entry-imu-stop")
            return 0.0, 0.0
        heading = self._turn_wait_heading
        if heading is None:
            # 常规入口已由 _sync_bridge 在 done 转换时设置；仅补齐无参考的状态。
            heading = self._turn_wait_heading = float(b.yaw)
        if not math.isfinite(heading):
            self._publish_zero("turn_wait_heading_invalid")
            self._reset_post_bridge_speed("entry-imu-stop")
            return 0.0, 0.0
        error = math.radians(b.angle_delta(heading, b.yaw))
        angular = float(np.clip(TURN_WAIT_HEADING_KP*error,
                                -TURN_ENTRY_MAX_W, TURN_ENTRY_MAX_W))
        linear = float(TURN_WAIT_SPEED)
        self._reset_post_bridge_speed("entry-yaw")
        self._post_bridge_speed_target = linear
        if self.enable_motion:
            command = self._rate_limit(linear, angular, now)
            self._publish(*command)
            self._last_command, self._last_command_valid = command, True
        else:
            self._publish_zero("motion_disabled")
            command = linear, angular
        return command


    def _follow_command(
        self, result: DetectionResult, now: float
    ) -> Tuple[float, float]:
        if self._in_right_turn() and result.curve_mode and not result.turn_entry_ready:
            return self._turn_wait_command(now)
        if self._have_error:
            filtered = self.error_filter_alpha * result.error + (
                1.0 - self.error_filter_alpha
            ) * self._previous_error
            dt = max(1e-3, now - self._previous_control_time)
            derivative = float(
                np.clip(
                    (filtered - self._previous_error) / dt,
                    -self.max_error_derivative,
                    self.max_error_derivative,
                )
            )
        else:
            filtered = result.error
            derivative = 0.0
            self._have_error = True
        self._previous_error = filtered
        self._previous_control_time = now
        angular = self.steering_sign * (
            self.steering_kp * filtered + self.steering_kd * derivative
        )
        angular = float(
            np.clip(angular, -self.max_angular_speed, self.max_angular_speed)
        )
        turn_ratio = min(1.0, abs(angular) / self.max_angular_speed)
        scale = self._speed_scale(now)
        slowdown = self.curve_slowdown
        linear = max(
            self.minimum_forward_speed * scale,
            self.forward_speed * scale * (1.0 - slowdown * turn_ratio),
        )
        if self._in_right_turn():
            linear = self._post_bridge_speed(result, angular, now)
        target = (linear, angular)
        if self.enable_motion:
            command = self._rate_limit(*target, now)
            self._publish(*command)
            self._last_command = command
            self._last_command_valid = True
        else:
            self._publish_zero("motion_disabled")
            command = target
        return command

    def _hold_sidewalk_stop(self, result: DetectionResult, now: float) -> None:
        if self.enable_motion:
            command = self._rate_limit(0.0, 0.0, now)
            self._publish(*command)
            self._last_command = command
            self._last_command_valid = True
        else:
            command = (0.0, 0.0)
            self._publish(0.0, 0.0)
            self._last_command = command

        stopped = abs(command[0]) <= 0.02 and abs(command[1]) <= 0.04
        if stopped or not self.enable_motion:
            if self._stable_since is None:
                self._stable_since = now
                self._announce(
                    f"人行道已停稳，开始计时 {self.sidewalk_stop_seconds:.0f} 秒"
                )
            elif now - self._stable_since >= self.sidewalk_stop_seconds:
                self._mode = self.MODE_FOLLOW
                self._passed_sidewalk = True
                self._stable_since = None
                self._have_error = False
                self._announce("人行道停车结束，继续出发")
        else:
            self._stable_since = None

        self._target_height = self.stand_height
        self._update_height(now)
        elapsed = 0.0 if self._stable_since is None else now - self._stable_since
        self._show(
            result,
            *command,
            self.enable_motion,
            None,
            extra=f"STOP {elapsed:.1f}/{self.sidewalk_stop_seconds:.0f}s",
        )

    def _hold_finish_stop(self, result: DetectionResult, now: float) -> None:
        elapsed = 0.0 if self._finish_since is None else now - self._finish_since
        driving_in = elapsed < FINISH_DRIVE_IN_SECONDS
        if driving_in:
            target = (FINISH_DRIVE_IN_SPEED, 0.0)
            extra = f"FINISH IN {elapsed:.1f}/{FINISH_DRIVE_IN_SECONDS:.1f}s"
        else:
            target = (0.0, 0.0)
            extra = "FINISH STOP"
        if self.enable_motion:
            command = self._rate_limit(*target, now)
            self._publish(*command)
            self._last_command = command
            self._last_command_valid = True
        else:
            command = target
            self._publish(0.0, 0.0)
            self._last_command = command

        self._target_height = self.stand_height
        self._update_height(now)
        self._show(
            result,
            *command,
            self.enable_motion,
            None,
            extra=extra,
        )

    def _in_right_turn(self) -> bool:
        return self._cleared_bridge and not self._passed_sidewalk

    def _has_left_foam(self, result: DetectionResult) -> bool:
        """只有黄线在画面左半、行数够，才算真正的外侧左海绵。"""
        if result.curve_mode:
            return (result.valid and result.boundary_side in ("left", "both")
                    and result.measured_points >= CURVE_MIN_POINTS
                    and result.boundary_span_pixels >= CURVE_MIN_SPAN_RATIO*self.image_width)
        if not result.valid or result.left_line_rows < 18:
            return False
        max_x = TURN_LEFT_MAX_X_RATIO * float(self.image_width)
        return 8.0 < result.left_edge_x < max_x

    def _usable_turn_line(self, result: DetectionResult) -> bool:
        """允许可靠左线、双线或单侧后备线；不是看到任意黄色就前进。"""
        return (
            result.valid and math.isfinite(result.error)
            and math.isfinite(result.confidence)
            and result.confidence >= TURN_FALLBACK_MIN_CONFIDENCE
            and ((result.curve_mode and result.measured_points >= CURVE_MIN_POINTS
                  and result.boundary_span_pixels >= CURVE_MIN_SPAN_RATIO*self.image_width)
                 or (not result.curve_mode and max(result.left_line_rows, result.right_line_rows) >= TURN_FALLBACK_MIN_ROWS))
            and (abs(result.error) <= 1.0 if result.curve_mode else abs(result.error) < TURN_FALLBACK_MAX_ERROR)
            and result.reason in ("left_line", "tracking", "single_line")
        )

    def _detect_lane(self, frame: np.ndarray) -> DetectionResult:
        # WAIT 起步后 TURN_WAIT_MIN_SECONDS 秒内不允许入弯（过期即为 0）。
        wait_elapsed = (
            0.0 if self._cleared_at is None else time.monotonic() - self._cleared_at
        )
        self.line_detector.turn_entry_hold = (
            max(0.0, TURN_WAIT_MIN_SECONDS - wait_elapsed)
            if self._cleared_bridge else 0.0
        )
        # 连续内沿检测内部已处理左右线选择，不需要再扫描一次。
        if self.line_detector.column_scan:
            return self.line_detector.detect(frame)
        if not self._in_right_turn():
            self._turn_fallback_active = False
            return self.line_detector.detect(frame)
        def detect_normal():
            saved = self.line_detector.force_left_line
            self.line_detector.force_left_line = False
            try:
                return self.line_detector.detect(frame)
            finally:
                self.line_detector.force_left_line = saved
        fallback = None
        if self._turn_fallback_active:
            fallback = detect_normal()
            if self._usable_turn_line(fallback):
                # 持续只见右线时每帧只算一次；左线重新可用，下一帧切回左线优先。
                if self._has_left_foam(fallback):
                    self._turn_fallback_active = False
                return fallback
        result = self.line_detector.detect(frame)
        if self._has_left_foam(result) and self._usable_turn_line(result):
            self._turn_fallback_active = False
            return result
        # 强制左线会把唯一右线也命名为左线，再被位置门限挡住。
        # 左线不可用时才用原双线/左右单线算法，避免仍有边线却原地扫线。
        if self.line_detector.force_left_line:
            result = fallback if fallback is not None else detect_normal()
        self._turn_fallback_active = True
        if not self._usable_turn_line(result):
            result.valid = False
        return result

    def _should_search_after_bridge(
        self, result: DetectionResult, now: float
    ) -> bool:
        """确认离桥后有可用线就跟随；不要求必须先锁到左线。"""
        if not self._in_right_turn() or self._cleared_at is None:
            return False
        if self._usable_turn_line(result):
            if self._has_left_foam(result) and not self._left_line_locked:
                self._left_line_locked = True
                self._announce("已锁到左侧海绵，接续桥后循迹")
            if self._turn_search_timed_out:
                self._announce("扫线停车后重新发现可用边线：恢复低速循迹")
            self._turn_search_timed_out = False
            self._turn_lost_announced = False
            return False
        # 不允许 valid=True 的低质量/饱和误差绕过后面的丢线处理。
        result.valid = False
        return not self._left_line_locked

    def _handle_lost_line(
        self,
        result: DetectionResult,
        now: float,
        searching: bool = False,
    ) -> None:
        turning = self._in_right_turn()
        if turning:
            self._reset_post_bridge_speed("lost")
        if self._lost_since is None:
            self._lost_since = now
        lost_for = now - self._lost_since
        extra = ""
        if (turning and self.line_detector.column_scan
                and not self.line_detector._curve_turn_entry_ready):
            # 尚未到弯口时丢掉全部可靠边线，不允许旧扫线分支绕过入弯限转。
            self._publish_zero("turn_entry_no_line")
            self._update_height(now)
            self._show(result, 0.0, 0.0, False, extra="ENTRY WAIT / NO RELIABLE LINE")
            return
        # 隧道里白墙/反光经常丢黄线：一直慢速往前拱，不要停在洞口。
        if self._mode == self.MODE_CROUCH:
            tunnel_v = self.forward_speed * self.tunnel_speed_scale
            if self._last_command_valid:
                creep_v = float(
                    np.clip(
                        abs(self._last_command[0]),
                        self.minimum_forward_speed,
                        tunnel_v,
                    )
                )
                creep_w = float(np.clip(self._last_command[1], -0.10, 0.10))
            else:
                creep_v = max(self.minimum_forward_speed, 0.85 * tunnel_v)
                creep_w = 0.0
            extra = f"TUNNEL CREEP {lost_for:.1f}s"
            if self.enable_motion:
                command = self._rate_limit(creep_v, creep_w, now)
                self._publish(*command)
                self._last_command = command
                self._last_command_valid = True
            else:
                command = (creep_v, creep_w)
                self._publish_zero(result.reason)
            self._update_height(now)
            self._show(
                result,
                *command,
                self.enable_motion,
                extra=extra,
            )
            self._log(now, f"丢线 {lost_for:.1f} 秒 lost_tunnel {result.reason}")
            return
        if turning:
            grace = TURN_LOST_GRACE_SECONDS
        elif self._passed_tunnel:
            grace = POST_TUNNEL_LOST_GRACE_SECONDS
        else:
            grace = self.lost_line_grace_seconds
        search_limit = TURN_SEARCH_SECONDS if searching else grace
        if self.enable_motion and lost_for < search_limit:
            # 已锁到左侧海绵后，丢线不再"停住+原地转"，改为重发上一条指令保持前进与姿态（于萧阳 10.29）。
            # 未锁定时维持原有搜索行为不变。
            if (turning and not self._left_line_locked) or searching:
                elapsed = (
                    0.0
                    if self._cleared_at is None
                    else now - self._cleared_at
                )
                if searching and elapsed < TURN_STOP_SECONDS:
                    target = (0.0, 0.0)
                    extra = f"TURN STOP {elapsed:.1f}s"
                    if not self._turn_lost_announced:
                        self._turn_lost_announced = True
                        self._announce("过完桥先停车，接着向左慢转找左侧海绵")
                else:
                    # 往泡沫所在的一侧转，才能把它拉回画面中间。
                    # 实测 shiwai_20261023_174832.avi：err 卡在 +0.97，
                    # 用 force_left_line 反推泡沫内沿在 x≈489（画面右侧），
                    # 却一直向左转 6.7 秒；左转会让场景整体右移，泡沫越转越靠右，
                    # 永远满足不了 left_edge_x < 269，于是永远锁不上。
                    if result.left_edge_x > 0.5 * self.image_width:
                        self._turn_search_dir = -1.0
                    elif 8.0 < result.left_edge_x < 0.5 * self.image_width:
                        self._turn_search_dir = 1.0
                    # 完全没看到左缘时沿用上一次的方向，不要来回抖。
                    target = (
                        TURN_LOST_SPEED,
                        self._turn_search_dir * TURN_LOST_ANGULAR,
                    )
                    side = "L" if self._turn_search_dir > 0.0 else "R"
                    extra = (
                        f"TURN SEARCH {side} {elapsed:.1f}s"
                        if searching
                        else f"TURN LOST {side} {lost_for:.1f}s"
                    )
                    if not self._turn_lost_announced:
                        self._turn_lost_announced = True
                        self._announce("桥后没有可用边线：按边线位置选择方向，限时原地找线")
                command = self._rate_limit(*target, now)
                self._publish(*command)
                self._last_command = command
                self._last_command_valid = True
                reason = f"lost_turn {result.reason}"
            elif self._last_command_valid:
                self._publish(*self._last_command)
                command = self._last_command
                reason = f"lost_hold {result.reason}"
            else:
                self._publish_zero(result.reason)
                command = (0.0, 0.0)
                reason = result.reason
        else:
            self._publish_zero(result.reason)
            command = (0.0, 0.0)
            reason = result.reason
            if turning and self.enable_motion and lost_for >= search_limit:
                extra = f"SEARCH TIMEOUT {lost_for:.1f}s / WAIT LINE"
                if not self._turn_search_timed_out:
                    self._turn_search_timed_out = True
                    self._announce(
                        f"扫线超过 {search_limit:.1f} 秒，停止旋转；继续识别，出现可用边线后自动恢复"
                    )
        self._update_height(now)
        self._show(
            result,
            *command,
            self.enable_motion and command != (0.0, 0.0),
            extra=extra,
        )
        self._log(now, f"丢线 {lost_for:.1f} 秒 {reason}")

    def _show(
        self,
        result: DetectionResult,
        linear: float,
        angular: float,
        command_active: bool,
        scene: Optional[SceneObservation] = None,
        extra: str = "",
    ) -> None:
        if self._bridge.state == "fault":
            state, color, _label = "BRIDGE FAULT", (0, 0, 255), "cmd"
        elif self._mode == self.MODE_STOP:
            state, color, _label = "SIDEWALK STOP", (220, 220, 220), "cmd"
        elif self._mode == self.MODE_FINISH:
            driving_in = (
                self._finish_since is not None
                and (time.monotonic() - self._finish_since) < FINISH_DRIVE_IN_SECONDS
            )
            state, color, _label = (
                ("FINISH IN", (255, 255, 255), "cmd")
                if driving_in
                else ("FINISH STOP", (255, 255, 255), "cmd")
            )
        elif self._mode == self.MODE_CROUCH:
            state, color, _label = "TUNNEL CROUCH", (200, 200, 200), "cmd"
        elif command_active:
            state, color, _label = "RUN", (255, 255, 255), "cmd"
        elif self.enable_motion:
            state, color, _label = "ARMED / STOP", (210, 210, 210), "cmd"
        else:
            state, color, _label = "STOP / DRY-RUN", (180, 180, 180), "planned"
        debug = result.debug_frame
        bar = debug[0:92].copy()
        debug[0:92] = cv2.addWeighted(bar, 0.35, np.zeros_like(bar), 0.65, 0.0)
        cv2.putText(
            debug,
            f"{state}  {result.reason}  conf={result.confidence:.2f}",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            f"err={result.error:+.3f}  v={linear:.2f}  ov={self._bridge.speed:.2f}  "
            f"w={angular:+.2f}  h={self._current_height:.2f}  "
            f"roll={self._current_roll:+.1f}  "
            f"pitch={self._current_pitch:+.1f}",
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        progress = (
            f"bridge={'Y' if self._passed_bridge else 'N'} "
            f"walk={'Y' if self._passed_sidewalk else 'N'} "
            f"tunnel={'Y' if self._passed_tunnel else 'N'} "
            f"finish={'Y' if self._passed_finish else 'N'}"
        )
        if (
            self._passed_tunnel
            and not self._passed_finish
            and self._passed_tunnel_at is not None
        ):
            wait_left = FINISH_AFTER_TUNNEL_SECONDS - (
                time.monotonic() - self._passed_tunnel_at
            )
            if wait_left > 0.0:
                progress += f" fn-wait {wait_left:.1f}s"
        if self._passed_bridge and not self._passed_sidewalk:
            b = self._bridge
            dr = 0.0 if b.baseline is None else b.angle_delta(b.roll, b.baseline[0])
            travel = 0.0 if b.entered_at is None else b.distance-b.entered_distance
            progress = f"bridge={b.state} droll={dr:+.1f} dist={travel:.2f}m"
            if b.state == "fault":
                progress += f" {b.reason}"
        if self._in_right_turn():
            progress += f" post={self._post_bridge_speed_mode} goal={self._post_bridge_speed_target:.2f}"
        if extra:
            progress = f"{progress} {extra}"
        cv2.putText(
            debug,
            progress,
            (12, 78),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (210, 230, 255),
            1,
            cv2.LINE_AA,
        )
        self._record_debug(debug)
        if not self._window_ready:
            return
        display = self._scale_debug_for_hdmi(debug) if DEBUG_FULLSCREEN else debug
        cv2.imshow(self.debug_window_name, display)
        if DEBUG_FULLSCREEN and not self._debug_fullscreen_applied:
            self._apply_debug_fullscreen()
            self._debug_fullscreen_applied = True
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            self.request_exit("operator_exit")

    @staticmethod
    def _detect_hdmi_size() -> Tuple[int, int]:
        try:
            output = subprocess.check_output(
                ["xrandr", "--current"],
                text=True,
                timeout=1.0,
            )
            match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
            if match:
                return int(match.group(1)), int(match.group(2))
            match = re.search(r"(\d+)x(\d+)\s+\S*\*", output)
            if match:
                return int(match.group(1)), int(match.group(2))
        except (OSError, subprocess.SubprocessError, ValueError):
            pass
        return 1920, 1080

    def _apply_debug_fullscreen(self) -> None:
        width, height = self._hdmi_size
        cv2.setWindowProperty(
            self.debug_window_name,
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN,
        )
        cv2.resizeWindow(self.debug_window_name, width, height)
        cv2.moveWindow(self.debug_window_name, 0, 0)

    def _scale_debug_for_hdmi(self, frame: np.ndarray) -> np.ndarray:
        screen_w, screen_h = self._hdmi_size
        height, width = frame.shape[:2]
        scale = min(screen_w / max(width, 1), screen_h / max(height, 1))
        new_w = max(1, int(width * scale))
        new_h = max(1, int(height * scale))
        scaled = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)
        x0 = (screen_w - new_w) // 2
        y0 = (screen_h - new_h) // 2
        canvas[y0 : y0 + new_h, x0 : x0 + new_w] = scaled
        return canvas

    def _open_video_writer(self, frame: np.ndarray) -> None:
        folder = Path(RECORDINGS_DIR)
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = folder / f"shiwai_{stamp}.avi"
        height, width = frame.shape[:2]
        self._video_size = (int(width), int(height))
        fps = max(5.0, float(self.control_rate))
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"MJPG"),
            fps,
            self._video_size,
        )
        if not writer.isOpened():
            self._announce("识别画面录像打开失败，这次只显示不保存")
            return
        self._video_writer = writer
        self._video_path = path
        self._video_frames = 0
        latest = folder / "shiwai_latest.avi"
        try:
            if latest.exists() or latest.is_symlink():
                latest.unlink()
            latest.symlink_to(path.name)
        except OSError:
            pass
        self._announce(f"开始录制识别画面：{path}")

    def _record_debug(self, frame: np.ndarray) -> None:
        if not RECORD_DEBUG_VIDEO or frame is None or frame.size == 0:
            return
        if self._video_writer is None:
            self._open_video_writer(frame)
        writer = self._video_writer
        if writer is None:
            return
        width, height = self._video_size
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, self._video_size)
        writer.write(frame)
        self._video_frames += 1

    def _close_video_writer(self) -> None:
        writer = self._video_writer
        self._video_writer = None
        if writer is None:
            return
        writer.release()
        path = self._video_path
        if path is None:
            self._announce("识别画面录像已关闭")
            return
        self._announce(
            f"识别画面已保存：{path}，共 {self._video_frames} 帧"
        )

    def _log_status(
        self, now: float, result: DetectionResult, scene: SceneObservation
    ) -> None:
        """1Hz 状态快照，让终端能直接看到状态机在干什么。

        _log() 只在出问题时打印（摄像头不可用/丢线），正常跑起来终端一片安静，
        看不出当前 mode、已经过了哪几关、场景分数在什么水平。
        这里每 status_log_period 秒打一行完整快照，调参和排查误触发都靠它。
        """
        if now - self._last_status < self.status_log_period:
            return
        self._last_status = now

        def mark(flag: bool) -> str:
            return "Y" if flag else "-"

        bridge_state = self._bridge.state
        progress = (
            f"bridge={bridge_state}"
            f" walk={mark(self._passed_sidewalk)}"
            f" tunnel={mark(self._passed_tunnel)}"
            f" finish={mark(self._passed_finish)}"
        )
        scenes = (
            f"hz={scene.hazard_score:.2f}{mark(scene.hazard)}"
            f" near={mark(scene.hazard_near)}"
            f" cw={scene.crosswalk_score:.2f}/{scene.stripe_count}"
            f"{mark(scene.crosswalk)}"
            f" fn={scene.finish_score:.2f}/{scene.finish_blobs}"
            f"{mark(scene.finish)}"
        )
        self.get_logger().info(
            f"[状态] {self._mode} {progress}"
            f" | 线 {result.reason} conf={result.confidence:.2f}"
            f" L={result.left_line_rows} R={result.right_line_rows}"
            f" err={result.error:+.3f}"
            f" 劲{result.turn_strength:.2f}/{self.line_detector._turn_ff_strength:.2f}"
            f" dir={result.heading_error:+.2f}"
            f" slope={result.heading_slope:+.2f}"
            f" exit={'Y' if self.line_detector._curve_exit_ready else 'N'}"
            f" | 场景 {scenes}"
            f" | h={self._current_height:.2f}"
            f" | imu_roll={self._bridge.roll:+.1f} pitch={self._bridge.pitch:+.1f}"
            f" base={self._bridge.baseline} odom_v={self._bridge.speed:.2f}"
            f" exit_good={self._bridge.exit_good_seconds:.2f}s"
            f" exit_hold={(now-self._bridge.exit_hold_since) if self._bridge.exit_hold_since is not None else 0.0:.2f}s"
            f" search_timeout={self._turn_search_timed_out}"
            f" exit_age={(now-self._bridge.flat_since) if self._bridge.flat_since is not None else 0.0:.2f}s"
            f" exit_dist={(self._bridge.distance-self._bridge.flat_distance) if self._bridge.flat_since is not None else 0.0:.2f}m"
            f" | post_speed={self._post_bridge_speed_mode} goal={self._post_bridge_speed_target:.2f}"
        )

    def _log(self, now: float, text: str) -> None:
        if now - self._last_log >= self.status_log_period:
            self.get_logger().info(text)
            self._last_log = now

    def request_exit(self, reason: str) -> None:
        self._publish_zero(reason)
        self._target_height = self.stand_height
        self._current_height = self.stand_height
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._publish_posture(self.stand_height)
        self._exit_requested = True

    @property
    def exit_requested(self) -> bool:
        return self._exit_requested

    def close(self) -> None:
        self._closing = True
        self._stop_capture.set()
        with self._capture_lock:
            capture = self._capture
        if capture is not None:
            capture.release()
        if hasattr(self, "_capture_thread"):
            self._capture_thread.join(timeout=1.0)
        if rclpy.ok():
            self._current_roll = 0.0
            self._current_pitch = 0.0
            for _ in range(10):
                self._publish(0.0, 0.0)
                self._publish_posture(self.stand_height)
                time.sleep(0.02)
        if self._window_ready:
            cv2.destroyWindow(self.debug_window_name)
        self._close_video_writer()
        self.get_logger().info("室外赛道节点退出，已重复发布零速度并恢复站立高度")


def main(args=None) -> None:
    node = None
    exit_event = threading.Event()

    def signal_handler(signum, _frame) -> None:
        exit_event.set()
        if node is not None:
            node.request_exit(f"signal_{signum}")

    previous_sigint = signal.signal(signal.SIGINT, signal_handler)
    previous_sigterm = signal.signal(signal.SIGTERM, signal_handler)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ShiWaiFollower()
        while rclpy.ok() and not node.exit_requested and not exit_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()