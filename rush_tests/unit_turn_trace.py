# 验证出弯判据 + 逐帧轨迹记录
#   1) 线持续可见 -> 4 帧锁定，且锁定点在 45° 之后
#   2) 只有 1~2 帧闪现 -> 不锁定（旧逻辑 2 帧就会锁 = 假锁）
#   3) 完全没线 -> 落到角度兜底 110°
#   4) 纯角度模式 -> 无视视觉，直接 110°
import importlib.util, sys, types, math

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); sys.modules[name] = m
    spec.loader.exec_module(m); return m

NEW = load("/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py", "tt_new")

RATE = 15.0
W = NEW.RUSH_TURN_W                  # -1.214 rad/s
DEG_PER_FRAME = abs(W) / RATE * 57.2958

def run(label, vis_frames, mode="vision", max_frames=40):
    NEW.RUSH_TURN_EXIT_MODE = mode
    bus = types.SimpleNamespace(yaw=0.0, speed=0.60, boost=False, state="done",
                                angle_delta=lambda a, b: a - b,
                              signed_distance=0.0)
    s = types.SimpleNamespace(
        _bridge=bus, control_rate=RATE, enable_motion=True,
        _turn_started_at=0.0, _turn_start_yaw=0.0, _turn_spooled=True,
        _turn_spool_started_at=0.0, _turn_spool_announced=False,
        _turn_low_speed_seconds=0.0, _turn_slow_warned=False,
        _turn_lock_frames=0, _turn_trace=[], _turn_frames=0,
        _turn_valid_frames=0, _turn_run_len=0, _turn_run_max=0,
        _turn_err_min=999.0,
        _turn_done_yaw=0.0, _turn_done_seconds=0.0,
        _last_published=(0.0, 0.0), _last_command=(0.0, 0.0),
        stand_height=0.25, _mode="TURN",
        MODE_TURN="TURN", MODE_SIDEWALK="SIDEWALK",
        _announce=lambda m: None, _update_height=lambda now: None,
        _show=lambda *a, **k: None,
        get_logger=lambda: types.SimpleNamespace(info=lambda m: LOG.append(m)),
    )
    LOG = []
    s.wheel_speeds = NEW.RushFollower.wheel_speeds
    for n in ("_turn_command", "_turn_vision_locked", "_dump_turn_trace",
              "_enter_sidewalk"):
        setattr(s, n, types.MethodType(getattr(NEW.RushFollower, n), s))
    s._emit = lambda lin, ang, now: (setattr(s, "_last_published", (lin, ang)),
                                     (lin, ang))[1]

    for i in range(max_frames):
        t = i / RATE
        bus.yaw = -abs(W) * t * 57.2958      # 右转，yaw 递减
        valid, rows, err = vis_frames.get(i, (False, 0, 0.0))
        res = types.SimpleNamespace(valid=valid, left_line_rows=rows,
                                    right_line_rows=rows, error=err)
        s._turn_command(res, t)
        if s._mode == "SIDEWALK":
            break
    summary = [m for m in LOG if "标定-出弯" in m]
    nf = len([m for m in LOG if "转弯帧" in m])
    print("  %-34s -> %s" % (label, summary[0].split("] ")[1] if summary else "没出弯 ✗"))
    print("  %-34s    轨迹打印 %d 行" % ("", nf))
    return s

print("每帧转 %.1f°，45° 在第 %d 帧" % (DEG_PER_FRAME, int(45 / DEG_PER_FRAME)))
print()
print("=== 1) 线从第 12 帧起持续可见（真锁定）===")
run("线持续可见", {i: (True, 24, 0.05) for i in range(12, 40)})

print()
print("=== 2) 假锁对照：只在第 12 帧闪 1 下 / 闪 2 下 ===")
run("闪 1 帧", {12: (True, 24, 0.05)})
run("闪 2 帧", {12: (True, 24, 0.05), 13: (True, 24, 0.05)})
run("闪 3 帧", {12: (True, 24, 0.05), 13: (True, 24, 0.05), 14: (True, 24, 0.05)})

print()
print("=== 3) 完全没线 ===")
run("没线", {})

print()
print("=== 4) 纯角度模式（无视视觉）===")
run("纯角度模式", {i: (True, 24, 0.05) for i in range(12, 40)}, mode="angle")

print()
print("=== 5) 线抖动（持续可见但行数不够）===")
run("行数不够 rows=10", {i: (True, 10, 0.05) for i in range(12, 40)})
run("偏差太大 err=0.50", {i: (True, 24, 0.50) for i in range(12, 40)})
