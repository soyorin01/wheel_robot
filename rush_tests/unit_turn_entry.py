# 直接调用真实 _turn_command + _rate_limit，模拟「过桥收尾 v=0.10，入弯触发立刻响」。
import importlib.util, sys, types

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); sys.modules[name] = m
    spec.loader.exec_module(m); return m

def make(mod, yaw_rate=100.0):
    log = []
    bus = types.SimpleNamespace(yaw=0.0, speed=0.10, boost=False, state="done",
                                signed_distance=0.0,
                                angle_delta=lambda a, b: ((a - b + 180.0) % 360.0) - 180.0)
    s = types.SimpleNamespace(
        enable_motion=True, control_rate=15.0,
        max_linear_acceleration=0.25, max_angular_acceleration=12.0,
        _last_command=(0.10, 0.0), _last_published=(0.10, 0.0), _last_motion_time=0.0,
        _mode=mod.RushFollower.MODE_TURN, stand_height=0.25, _target_height=0.25,
        _bridge=bus, _have_error=False,
        _turn_start_yaw=0.0, _turn_started_at=0.0, _turn_spool_started_at=0.0,
        _turn_spooled=False, _turn_spool_announced=False,
        _turn_low_speed_seconds=0.0, _turn_slow_warned=False,
        _turn_lock_frames=0, _turn_trace=[], _turn_frames=0,
        _turn_valid_frames=0, _turn_run_len=0, _turn_run_max=0,
        _turn_err_min=999.0,
        _turn_done_yaw=0.0, _turn_done_seconds=0.0,
        _sidewalk_started_at=None, _stop_sampled_at=0.0, _stop_speed_filter=0.0,
        _standstill_seconds=0.0, _moving_seconds=0.0,
        _announce=lambda m: log.append(m),
        _brake_hard=False,
        get_logger=lambda: types.SimpleNamespace(
            info=lambda m: log.append(m)),
        _update_height=lambda now: None, _show=lambda *a, **k: None,
        MODE_TURN=mod.RushFollower.MODE_TURN, MODE_SIDEWALK=mod.RushFollower.MODE_SIDEWALK,
        MODE_WAIT_TURN=getattr(mod.RushFollower, 'MODE_WAIT_TURN', 'WAIT_TURN'),
        _yaw_rate=yaw_rate,
    )
    s.wheel_speeds = mod.RushFollower.wheel_speeds
    s._approach = mod.RushFollower._approach                    # staticmethod
    if hasattr(mod.RushFollower, "_turn_vision_locked"):
        s._turn_vision_locked = types.MethodType(
            mod.RushFollower._turn_vision_locked, s)
    else:
        s._turn_vision_locked = lambda result: False
    for n in ("_rate_limit", "_turn_command", "_dump_turn_trace"):
        setattr(s, n, types.MethodType(getattr(mod.RushFollower, n), s))
    if hasattr(mod.RushFollower, "_enter_sidewalk"):
        s._enter_sidewalk = types.MethodType(mod.RushFollower._enter_sidewalk, s)
    else:
        def _es(now, _s=s):
            _s._mode = _s.MODE_SIDEWALK
        s._enter_sidewalk = _es
    def emit(linear, angular, now):
        if s.enable_motion:
            s._last_command = s._last_published = s._rate_limit(linear, angular, now)
        return s._last_command
    s._emit = emit
    return s, bus, log

_RES = types.SimpleNamespace(valid=False, left_line_rows=0,
                             right_line_rows=0, error=0.0,
                             debug_frame=None, reason="test")

def run(mod, label):
    if not hasattr(mod, "RUSH_TURN_V_READY_TOL"):
        mod.RUSH_TURN_V_READY_TOL = 0.0
        mod.RUSH_TURN_ENTRY_MAX_SECONDS = 0.0
        mod.RUSH_TURN_ENTRY_ACCEL = mod.RUSH_LINEAR_ACCEL
    s, bus, log = make(mod)
    dt, t, yaw = 1.0/15.0, 0.0, 0.0
    rot_start = v_at_rot = min_rw = None
    dist = 0.0
    for _ in range(int(8.0/dt)):
        s._turn_command(_RES, t)
        if abs(s._last_published[1]) > 0.10:      # 正在旋转
            if rot_start is None:
                rot_start, v_at_rot = t, s._last_published[0]
            yaw += s._yaw_rate * dt; bus.yaw = yaw
            rw = s.wheel_speeds(*s._last_published)[1]
            min_rw = rw if min_rw is None else min(min_rw, rw)
        else:
            dist += abs(s._last_published[0]) * dt
        bus.speed = abs(s._last_published[0])
        t += dt
        if s._mode != mod.RushFollower.MODE_TURN:
            break
    print("  %-6s 起转 %-7s 起转时 v=%-6s 转弯中右轮最小 %-7s 起转前直冲 %.2fm  全程 %.2fs" % (
        label,
        "%.2fs" % rot_start if rot_start else "立即",
        "%.2f" % v_at_rot if v_at_rot is not None else "-",
        "%+.2f" % min_rw if min_rw is not None else "-",
        dist, t))
    if min_rw is not None and min_rw < 0:
        print("         -> 右轮倒转：原地打转")
    else:
        print("         -> 右轮始终正转：沿弧线过弯")
    return min_rw

new = load("/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py", "rt_new")
try:
    old = load("/tmp/rush_before_turnfix.py", "rt_old")
except Exception:
    old = None
    print("  !! 旧版本备份 /tmp/rush_before_turnfix.py 不存在（Pi 重启清空了 /tmp），本文件的新旧对照跳过，只看新逻辑")

print("场景：过桥收尾 v=0.10 m/s，入弯触发立刻响（复现 19:37 那趟）")
print()
m_old = run(old, "旧逻辑") if old is not None else None
m_new = run(new, "新逻辑")
print()
print("对照：")
if m_old is not None:
    print("  旧逻辑 转弯中右轮最小 %+.2f m/s  -> %s"
          % (m_old, "原地打转（= 实车现象）" if m_old < 0 else "正常"))
print("  新逻辑 转弯中右轮最小 %+.2f m/s  -> %s" % (m_new, "原地打转" if m_new < 0 else "正常"))
