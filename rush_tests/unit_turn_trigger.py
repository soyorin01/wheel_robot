# 验证入弯判定的「最小里程」门槛，以及「桥后=X.XXm」有没有进日志/录像。
import importlib.util, sys, types

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); sys.modules[name] = m
    spec.loader.exec_module(m); return m

def stub(mod, travelled, entry_ready, min_distance, fallback=99.0):
    mod.RUSH_TURN_MIN_DISTANCE = min_distance
    mod.RUSH_TURN_START_DISTANCE = fallback
    mod.RUSH_TURN_TRIGGER = "visual"
    log = []
    bus = types.SimpleNamespace(distance=10.0 + travelled, yaw=-30.0, state="done",
                                speed=0.55, roll=0.0, pitch=0.0)
    s = types.SimpleNamespace(
        control_rate=15.0,
        _wait_turn_frames=0, _wait_turn_valid=0,
        _bridge=bus, _bridge_clear_distance=10.0, _cleared_bridge=True,
        _mode=mod.RushFollower.MODE_WAIT_TURN,
        _turn_start_yaw=None, _turn_started_at=None, _turn_spool_started_at=None,
        _turn_spooled=False, _turn_spool_announced=False,
        _turn_low_speed_seconds=0.0, _turn_slow_warned=False,
        _have_error=True, _passed_bridge=True, _passed_sidewalk=False,
        _passed_tunnel=False, _passed_finish=False, _current_height=0.25,
        _last_published=(0.55, 0.0), _guard_clamped_frames=0,
        _standstill_seconds=0.0, status_log_period=1.0, _last_status=0.0,
        _announce=lambda m: log.append(m),
        MODE_TURN=mod.RushFollower.MODE_TURN,
    )
    s.wheel_speeds = mod.RushFollower.wheel_speeds
    s._rush_command = lambda result, now, tag=None: log.append("直冲:" + str(tag))
    s.get_logger = lambda: types.SimpleNamespace(info=lambda m: log.append(m))
    result = types.SimpleNamespace(turn_entry_ready=entry_ready, reason="left_line",
                                   confidence=0.9, error=0.05, debug_frame=None,
                                   valid=True, left_line_rows=24, right_line_rows=24)
    for n in ("_wait_turn_command", "_log_status"):
        setattr(s, n, types.MethodType(getattr(mod.RushFollower, n), s))
    scene = types.SimpleNamespace(hazard_score=0.0, hazard=False, finish_score=0.0,
                                  finish_blobs=0, finish=False)
    return s, result, scene, log

new = load("/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py", "tt_new")
try:
    old = load("/tmp/rush_before_turnfix.py", "tt_old")
except Exception:
    old = None
    print("  !! 旧版本备份 /tmp/rush_before_turnfix.py 不存在（Pi 重启清空了 /tmp），本文件的新旧对照跳过，只看新逻辑")


print("=== 1) 入弯门槛：视觉已确认(entry=Y)，但里程不够 ===")
print("   场景：视觉确认在离桥后 0.27 m 就亮了（实车就是这样）")
for md in (0.00, 0.50, 1.00, 1.50):
    s, r, sc, log = stub(new, 0.27, True, md)
    s._wait_turn_command(r, 0.0)
    fired = s._mode == new.RushFollower.MODE_TURN
    print("   RUSH_TURN_MIN_DISTANCE=%.2f  -> %s   %s"
          % (md, "起转" if fired else "继续直冲", log[0][:62]))
print()
print("=== 2) 里程到了、视觉也确认了 ===")
s, r, sc, log = stub(new, 1.05, True, 1.00)
s._wait_turn_command(r, 0.0)
print("   桥后 1.05 m 门槛 1.00 -> %s" % ("起转 ✓" if s._mode == new.RushFollower.MODE_TURN else "没转 ✗"))
print()
print("=== 3) 里程到了但视觉没确认（不该转）===")
s, r, sc, log = stub(new, 1.50, False, 1.00)
s._wait_turn_command(r, 0.0)
print("   entry=- -> %s" % ("起转 ✗错了" if s._mode == new.RushFollower.MODE_TURN else "继续直冲 ✓"))
print()
print("=== 4) 日志里的「桥后」字段 ===")
s, r, sc, log = stub(new, 1.23, False, 1.00)
s._log_status(5.0, r, sc)
line = [m for m in log if m.startswith("[RUSH]")][0]
print("   " + " ".join(line.split())[-95:])
print("   含 '桥后= 1.23m' -> %s" % ("✓" if "桥后= 1.23m" in line else "✗"))
print()
print("=== 5) 兜底：视觉始终不确认，到最晚点必须强制转 ===")
for dist, fb in ((0.50, 0.90), (0.89, 0.90), (0.95, 0.90)):
    s, r, sc, log = stub(new, dist, False, 0.60, fb)
    s._wait_turn_command(r, 0.0)
    fired = s._mode == new.RushFollower.MODE_TURN
    print("   桥后 %.2f m 最晚 %.2f 视觉=-  -> %s"
          % (dist, fb, "起转 ✓(兜底)" if fired else "继续直冲"))
print()
if old is not None:
    print("=== 6) 旧逻辑没有这个门槛（对照）===")
    s, r, sc, log = stub(old, 0.27, True, 0.0)
    s._wait_turn_command(r, 0.0)
    print("   旧逻辑 桥后 0.27 m 视觉确认 -> %s（提前转）"
          % ("起转" if s._mode == old.RushFollower.MODE_TURN else "没转"))
