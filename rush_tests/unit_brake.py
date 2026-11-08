#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""出弯硬刹单元测试：验证「转到角度立刻收油」的刹车距离。

对照三组：
  A 旧行为（_brake_hard=False，只靠 0.25×3.0=0.75 m/s²）
  B 新行为（_brake_hard=True，RUSH_SIDEWALK_BRAKE_ACCEL）
  C 角速度刹车（应该一直很快，两帧内归零）
"""
import importlib.util
import sys
import types

SRC = "/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def make(mod, brake, v0, w0):
    bus = types.SimpleNamespace(boost=False, speed=v0, yaw=0.0)
    s = types.SimpleNamespace(
        control_rate=15.0,
        max_linear_acceleration=float(mod.RUSH_LINEAR_ACCEL),
        max_angular_acceleration=float(mod.RUSH_ANGULAR_ACCEL),
        _bridge=bus,
        _mode=mod.RushFollower.MODE_SIDEWALK,
        _last_command=(v0, w0),
        _last_motion_time=0.0,
        _brake_hard=brake,
        MODE_TURN=mod.RushFollower.MODE_TURN,
        MODE_WAIT_TURN=mod.RushFollower.MODE_WAIT_TURN,
    )
    s._approach = mod.RushFollower._approach
    s._rate_limit = types.MethodType(mod.RushFollower._rate_limit, s)
    return s


def run(mod, brake, v0=None, w0=None):
    v0 = float(mod.RUSH_TURN_V) if v0 is None else v0
    w0 = float(mod.RUSH_TURN_W) if w0 is None else w0
    s = make(mod, brake, v0, w0)
    dt = 1.0 / 15.0
    trace = [(0.0, v0, w0)]
    dist = 0.0
    t = 0.0
    for _ in range(60):
        t += dt
        v, w = s._rate_limit(0.0, 0.0, t)
        s._last_command = (v, w)          # _rate_limit 不更新它，_emit 才更新
        dist += max(0.0, (trace[-1][1] + v) / 2.0) * dt   # 梯形积分
        trace.append((t, v, w))
        if abs(v) < 1e-9 and abs(w) < 1e-9:
            break
    stop_t = next((x[0] for x in trace if abs(x[1]) < 1e-9), None)
    w_stop_t = next((x[0] for x in trace if abs(x[2]) < 1e-9), None)
    return trace, dist, stop_t, w_stop_t


def main():
    mod = load(SRC, "rt_brake")
    a_acc = float(mod.RUSH_LINEAR_ACCEL) * 3.0
    b_acc = float(mod.RUSH_SIDEWALK_BRAKE_ACCEL)

    print("=" * 68)
    print("出弯硬刹测试  rush.py")
    print("=" * 68)
    print("  RUSH_TURN_V = %.2f m/s   RUSH_TURN_W = %+.3f rad/s" %
          (mod.RUSH_TURN_V, mod.RUSH_TURN_W))
    print("  旧减速度 = RUSH_LINEAR_ACCEL(%.2f) x 3.0 = %.2f m/s²" %
          (mod.RUSH_LINEAR_ACCEL, a_acc))
    print("  新减速度 = RUSH_SIDEWALK_BRAKE_ACCEL = %.2f m/s²" % b_acc)
    print("  角减速度 = RUSH_ANGULAR_ACCEL = %.2f rad/s²" % mod.RUSH_ANGULAR_ACCEL)
    print()

    ok = True
    ta, da, sa, wa = run(mod, False)
    tb, db, sb, wb = run(mod, True)

    print("  A) 旧行为（_brake_hard=False）")
    print("     线速度归零 %.2fs   滑行 %.2f m   角速度归零 %.2fs" % (sa, da, wa))
    print("     " + "  ".join("f%d v=%.2f" % (i, x[1]) for i, x in enumerate(ta[:8])))
    print()
    print("  B) 新行为（_brake_hard=True）")
    print("     线速度归零 %.2fs   滑行 %.2f m   角速度归零 %.2fs" % (sb, db, wb))
    print("     " + "  ".join("f%d v=%.2f" % (i, x[1]) for i, x in enumerate(tb[:8])))
    print()

    if sb is None:
        print("  !! 新行为 1 秒内没停下来"); ok = False
    if db >= da:
        print("  !! 新行为没有比旧行为刹得短"); ok = False
    if sb is not None and sb > 0.60:
        print("  !! 新行为 %.2fs 刹得太慢，跟旧行为没区别" % sb); ok = False
    if sb is not None and sb < 0.15:
        print("  !! 新行为 %.2fs 就归零 = 一帧砸停（减速度太狠，车身会点头）" % sb)
        ok = False
    if db > 0.16:
        print("  !! 新行为滑行 %.2f m 太远（阈值 0.16 m）" % db)
        ok = False
    if wb is None or wb > 0.20:
        print("  !! 角速度刹车太慢: %s" % wb); ok = False

    # 角速度必须够快：转弯半径 0.49 m，w 归零期间多转的角度
    import math
    w0 = float(mod.RUSH_TURN_W)
    extra_deg = math.degrees(w0 ** 2 / (2.0 * float(mod.RUSH_ANGULAR_ACCEL)))
    print("  角速度刹车多转 %.1f°（应 < 6°，否则出弯角度会飘）" % extra_deg)
    if extra_deg >= 6.0:
        print("  !! 角速度刹车太软"); ok = False
    print()

    # 关键对比：用户实测那一趟
    print("  对比 11-08 10:36 实车录像：")
    print("     出弯后帧 110~113 命令 v 已降到 0.31~0.48，但实测 ov 还是 0.59~0.60")
    print("     录像结束（帧 117，出弯后 0.60s）v=0.09 ov=0.08 —— 还在滑")
    print("     新行为 %.2fs 就归零，比旧行为快 %.2f 倍" %
          (sb if sb else -1, (sa / sb) if (sa and sb) else -1))
    print()
    print("=" * 68)
    print("结果：%s" % ("全部通过" if ok else "有问题，见上面 !!"))
    print("=" * 68)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
