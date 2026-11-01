# 复现 19:45 那趟：起步基线在平地上学，过桥后 dp 永远 > 5°，3 秒后原逻辑故障停车。
# 验证新逻辑的兜底能把它按「已离桥」救回来，同时不会在真的没下桥时乱救。
import importlib.util, math, sys, types

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); sys.modules[name] = m
    spec.loader.exec_module(m); return m

def quat(roll, pitch, yaw):
    r, p, y = (math.radians(v) / 2.0 for v in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)

def simulate(mod, label, baseline_roll=-1.1, baseline_pitch=-1.6):
    b = mod.BridgeTraversal()
    now = [0.0]; dt = 1.0/50.0
    def step(roll, pitch, speed=0.60, yaw=0.0, gyro=(0.0, 0.0, 0.0)):
        now[0] += dt
        b.imu(quat(roll, pitch, yaw), gyro, now[0])
        b.odom(speed, now[0])
        b.tick(now[0])
    b.arm(now[0])
    # 1) 平地静置学基线（roll/pitch 必须真的安静才学得到）
    for _ in range(int(1.4/dt)):
        step(baseline_roll, baseline_pitch, speed=0.0)
    assert b.baseline is not None, "基线没学到"
    # 2) 直行接近，上桥
    for _ in range(int(0.4/dt)):
        step(baseline_roll, baseline_pitch)
    for _ in range(int(0.9/dt)):
        step(-12.5, 10.6)                      # 桥上：横滚 -12.5°
    # 3) 下桥：横滚回正，但俯仰停在 +7.9（实车就是这样）
    for _ in range(int(6.0/dt)):
        if b.state in ("done", "fault"):
            break
        step(-0.6, 7.9, speed=0.20)
    return b, now[0]

def check(mod, label, path):
    b, t = simulate(mod, label)
    print("  %-6s 基线 roll/pitch = %+.1f/%+.1f   过桥后 pitch=%+.1f  状态=%-6s  %s"
          % (label, b.baseline[0], b.baseline[1], b.pitch, b.state,
             "%.2f m" % b.distance))
    if b.state != "fault":
        print("         （没走到故障，说明这趟本来就能过）")
        return None
    logs = []
    s = types.SimpleNamespace(_bridge=b, _log=logs)
    s.get_logger = lambda: types.SimpleNamespace(warn=lambda m: logs.append(m))
    rescue = getattr(mod.RushFollower, "_bridge_exit_rescue", None)
    if rescue is None:
        print("         -> 旧逻辑没有兜底：硬停车，整趟报废")
        return False
    s._bridge_exit_rescue = types.MethodType(rescue, s)
    r = s._bridge_exit_rescue()
    print("         -> 兜底判定：%s" % ("救回（按已离桥继续）" if r else "不救（照旧停车）"))
    for m in logs:
        print("            " + m)
    return r

new = load("/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py", "br_new")
old = load("/tmp/rush_before_rescue.py", "br_old")
print("场景：起步在平地学基线(pitch=-1.6)，过桥后 pitch 停在 +7.9（复现 19:45）")
print()
r_old = check(old, "旧逻辑", None)
print()
r_new = check(new, "新逻辑", None)
print()
if r_new:
    b, t = simulate(new, "x")
    s = types.SimpleNamespace(_bridge=b, _log=[])
    s.get_logger = lambda: types.SimpleNamespace(warn=lambda m: None)
    s._bridge_exit_rescue = types.MethodType(new.RushFollower._bridge_exit_rescue, s)
    print("兜底的边界（不该乱救的情况）：")
    b.roll = -8.0                                   # 横滚没回正 = 还在桥上
    print("  横滚还在 -8.0°（像还在桥上）      -> %s" % ("救" if s._bridge_exit_rescue() else "不救 ✓"))
    b.roll = -0.6
    keep, b.distance = b.distance, b.entered_distance + 0.50
    print("  进桥后只走了 0.50 m               -> %s" % ("救" if s._bridge_exit_rescue() else "不救 ✓"))
    b.distance = keep
    b.pitch = 20.0
    print("  俯仰差 20°（歪得离谱/可能翻了）   -> %s" % ("救" if s._bridge_exit_rescue() else "不救 ✓"))
    b.pitch = 7.9
    b.reason = "IMU/odom stale or level baseline unavailable"
    print("  别的故障原因（传感器断了）        -> %s" % ("救" if s._bridge_exit_rescue() else "不救 ✓"))
print()
print("判定：旧逻辑 %s，新逻辑 %s" % (
    "故障硬停车" if r_old is False else "未知",
    "救回继续跑" if r_new else "没救回来"))
