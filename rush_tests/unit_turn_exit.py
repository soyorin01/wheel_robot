# 验证两件事：
#   1) 下桥基准点改成「固定里程」判定（不再随速度漂）
#   2) 出弯看视觉锁定（王赛的逻辑）
import importlib.util, sys, types

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); sys.modules[name] = m
    spec.loader.exec_module(m); return m

new = load("/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py", "te_new")
try:
    old = load("/tmp/rush_before_turnfix.py", "te_old")
except Exception:
    old = None
    print("  !! 旧版本备份 /tmp/rush_before_turnfix.py 不存在（Pi 重启清空了 /tmp），本文件的新旧对照跳过，只看新逻辑")


print("=== 1) 下桥基准点：按固定里程 ===")
print("   常量 BRIDGE_SETTLE_DISTANCE=%.2f  TIMEOUT=%.2f"
      % (new.BRIDGE_SETTLE_DISTANCE, new.BRIDGE_SETTLE_TIMEOUT))

def settle(mod, distances, times):
    bus = types.SimpleNamespace(distance=0.0, baseline=(-1.1, -1.6), roll=0.0,
                                pitch=0.0, roll_rate=0.0, imu_at=0.0, yaw=0.0)
    s = types.SimpleNamespace(_bridge=bus, _bridge_done_at=None,
                              _bridge_done_distance=0.0,
                              _bridge_settle_since=None)
    s._bridge_settle_ready = types.MethodType(mod.RushFollower._bridge_settle_ready, s)
    out = []
    for d, t in zip(distances, times):
        bus.distance = d
        out.append((d, t, s._bridge_settle_ready(t)))
    return out

# 第一帧 done 在 dist=2.00, now=0
rows = settle(new, [2.00, 2.05, 2.10, 2.24, 2.26], [0.0, 0.1, 0.2, 1.0, 1.1])
for d, t, r in rows:
    print("   dist=%.2f  t=%.2fs  -> %s" % (d, t, "放行" if r else "继续稳车身"))
# 车轮打滑：里程不涨，靠超时兜底
rows = settle(new, [2.00, 2.00, 2.00, 2.00], [0.0, 1.0, 2.0, 2.6])
print("   里程卡住不动（打滑）：t=%.1fs -> %s"
      % (rows[-1][1], "放行（超时兜底）" if rows[-1][2] else "卡死 ✗"))
print("   对照旧逻辑（只按时间）：2.6s 才放行，中间那段车在低速爬，起点随速度漂")

print()
print("=== 2) 出弯视觉锁定（王赛逻辑）===")
def lock(mod, seq):
    s = types.SimpleNamespace(_turn_lock_frames=0)
    s._turn_vision_locked = types.MethodType(mod.RushFollower._turn_vision_locked, s)
    out = []
    for valid, rows_, err in seq:
        r = types.SimpleNamespace(valid=valid, left_line_rows=rows_,
                                  right_line_rows=rows_, error=err)
        out.append(s._turn_vision_locked(r))
    return out

cases = [
    ("线回来且在中间（两次连续）", [(True, 22, 0.10)] * 2),
    ("线回来但偏得太远 err=0.50", [(True, 22, 0.50)] * 3),
    ("线行数不够 rows=10", [(True, 10, 0.05)] * 3),
    ("没线 valid=False", [(False, 25, 0.00)] * 3),
    ("闪一下又没了（不连续）", [(True, 22, 0.10), (False, 22, 0.10), (True, 22, 0.10)]),
]
for name, seq in cases:
    out = lock(new, seq)
    print("   %-26s -> %s" % (name, "锁定" if any(out) else "不锁定"))
print()
print("   对照旧逻辑（只看角度）：没有视觉判据，只能靠 RUSH_TURN_YAW_DEG 硬切")
print("   新逻辑角度上限 = %.0f°（原 90°），最小盲转 = %.0f°"
      % (new.RUSH_TURN_YAW_DEG, new.RUSH_TURN_LOCK_MIN_YAW_DEG))
