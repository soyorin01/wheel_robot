#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车位置保持单元测试。

底盘模型（一阶跟随 + 恒定后退扰动）：
    speed += ((cmd - D) - speed) * dt / tau
D = 0.28 m/s 是实测的后退扰动：只发 v=0 时底盘稳不住，会以约 0.28 m/s 往后溜。

两个变体：
  A) 老写法：出弯那一刻（车还在 0.60 m/s 往前冲）就锁目标点  -> 应该复现翻车
  B) 新写法：等车停稳（|speed| <= LATCH_SPEED）才锁目标点    -> 应该收敛
"""
import importlib.util
import sys

SRC = "/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py"

TAU = 0.28        # 底盘速度环时间常数（按实车滑行距离反推：收到 0 后还会再滑 ~0.2 m）
DT = 1.0 / 15.0
DIST = 0.28       # 实测后退扰动
V_EXIT = 0.60     # 出弯时车速


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def simulate(mod, latch_at_exit, seconds=8.0):
    # latch_at_exit=True 复现旧版：出弯就锁点 + 对称 clip（会主动倒车）
    gain = float(mod.RUSH_SIDEWALK_HOLD_GAIN)
    hmax = float(mod.RUSH_SIDEWALK_HOLD_MAX)
    db = float(mod.RUSH_SIDEWALK_HOLD_DEADBAND)
    latch_speed = float(mod.RUSH_SIDEWALK_HOLD_LATCH_SPEED)
    brake_acc = float(mod.RUSH_SIDEWALK_BRAKE_ACCEL)

    cmd = V_EXIT              # 出弯那一刻命令还是 0.60
    speed = V_EXIT
    signed = 0.0
    target = None
    latch_at = 0.0
    t = 0.0
    min_cmd = 0.0
    worst_back = 0.0          # 最大后退量（正数 = 退了多少米）
    trace = []

    if latch_at_exit:
        target = 0.0          # 出弯瞬间就锁

    for _ in range(int(seconds / DT)):
        t += DT

        # --- 刹车：命令按 1.5 m/s^2 从 0.60 收到 0 ---
        if cmd > 0.0:
            cmd = max(0.0, cmd - brake_acc * DT)

        # --- 位置保持 ---
        if gain > 0.0:
            if target is None:
                if not latch_at_exit and abs(speed) <= latch_speed:
                    target = signed
                    latch_at = t
            else:
                drift = target - signed
                if latch_at_exit:
                    # 旧版：出弯就锁点 + 对称 clip —— 目标点偏后就会主动倒车
                    cmd = max(-hmax, min(hmax, gain * drift))
                elif drift > db:
                    cmd = min(gain * drift, hmax)     # 新版：只往前顶
                else:
                    cmd = 0.0
        if cmd < min_cmd:
            min_cmd = cmd

        # --- 底盘 ---
        speed += ((cmd - DIST) - speed) * DT / TAU
        signed += speed * DT
        if target is not None and (target - signed) > worst_back:
            worst_back = target - signed
        trace.append((t, cmd, speed, signed, target, min_cmd))

    return trace, signed, min_cmd, worst_back


def main():
    mod = load(SRC, "rt_hold")
    print("=" * 72)
    print("停车位置保持测试  rush.py")
    print("=" * 72)
    print("  底盘模型：一阶跟随 (tau=%.2fs) + 后退扰动 %.2f m/s" % (TAU, DIST))
    print("  出弯速度 %.2f m/s，刹车 %.2f m/s^2" %
          (V_EXIT, mod.RUSH_SIDEWALK_BRAKE_ACCEL))
    print("  保持：增益 %.2f  上限 %.2f  死区 %.3f  锁点门槛 %.2f m/s" %
          (mod.RUSH_SIDEWALK_HOLD_GAIN, mod.RUSH_SIDEWALK_HOLD_MAX,
           mod.RUSH_SIDEWALK_HOLD_DEADBAND, mod.RUSH_SIDEWALK_HOLD_LATCH_SPEED))
    print()

    ok = True

    # ---- A) 老写法：出弯就锁点 ----
    tr, fin, minc, worst = simulate(mod, latch_at_exit=True)
    print("  A) 老写法（出弯那一刻就锁目标点）—— 应该复现 11:0x 的翻车")
    print("     出现过的最小指令 v = %+.3f m/s   （负 = 主动往后倒！）" % minc)
    print("     最大后退 %.3f m" % worst)
    if minc >= -0.02:
        print("     ?? 没复现出「主动倒车」，模型或逻辑有问题"); ok = False
    else:
        print("     OK 复现：控制器确实会主动发负速度把车倒回去")
    print()

    # ---- B) 新写法：停稳才锁点 ----
    tr, fin, minc, worst = simulate(mod, latch_at_exit=False)
    print("  B) 新写法（等车停稳才锁目标点 + 只往前顶）")
    print("     出现过的最小指令 v = %+.3f m/s   （必须 >= 0）" % minc)
    print("     最大后退 %.3f m   最终位置 %+.3f m" % (worst, fin))
    if minc < -1e-9:
        print("     !! 仍然发出了负速度，可能又把车倒回去"); ok = False
    if worst > 0.20:
        print("     !! 最大后退 %.3f m 太大（阈值 0.20 m）" % worst); ok = False
    print()
    print("     关键时刻（t / 指令v / 实际v / 里程 / 目标点）:")
    shown = 0
    for (t, c, sp, sg, tg, mc) in tr:
        if tg is not None and shown < 14:
            print("       t=%.2f  指令 %+.3f  实际 %+.3f  里程 %+.3f  目标 %+.3f"
                  % (t, c, sp, sg, tg))
            shown += 1
        if shown >= 14:
            break
    print("     最后 3 帧:")
    for (t, c, sp, sg, tg, mc) in tr[-3:]:
        print("       t=%.2f  指令 %+.3f  实际 %+.3f  里程 %+.3f  目标 %s"
              % (t, c, sp, sg, ("%+.3f" % tg) if tg is not None else "未锁"))
    print()

    # ---- C) 增益对比（用新写法）----
    print("  C) 增益对比（新写法）")
    base = float(mod.RUSH_SIDEWALK_HOLD_GAIN)
    for g in (0.0, 1.0, 3.0, 5.0, 8.0):
        mod.RUSH_SIDEWALK_HOLD_GAIN = g
        _, f, mc, w = simulate(mod, latch_at_exit=False)
        note = "  <- 现在" if abs(g - base) < 1e-9 else ("  <- 关" if g == 0 else "")
        print("     增益 %.1f -> 最终 %+.3f m  最大后退 %.3f m  最小指令 %+.3f%s"
              % (g, f, w, mc, note))
    mod.RUSH_SIDEWALK_HOLD_GAIN = base

    print()
    print("=" * 72)
    print("结果：%s" % ("全部通过" if ok else "有问题，见上面 !!"))
    print("=" * 72)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
