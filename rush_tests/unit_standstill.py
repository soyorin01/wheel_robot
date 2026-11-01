# 直接调用 rush.py 里真实的 _sidewalk_command 方法体，喂实测噪声。
# 不依赖相机、不依赖时序，确定性。
import importlib.util, random, sys, types

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod

def make_stub(mod, speed_seq):
    """构造一个只够 _sidewalk_command 跑的最小 self。"""
    log = []
    bus = types.SimpleNamespace(speed=0.0, yaw=0.0)
    self = types.SimpleNamespace(
        enable_motion=True,
        sidewalk_stop_seconds=5.0,
        control_rate=float(mod.RUSH_CONTROL_RATE) if hasattr(mod, "RUSH_CONTROL_RATE") else 15.0,
        stand_height=0.25, _target_height=0.25,
        _sidewalk_started_at=None, _stop_sampled_at=0.0,
        _stop_speed_filter=0.0, _standstill_seconds=0.0, _moving_seconds=0.0,
        _passed_sidewalk=False, _have_error=False, _mode="SIDEWALK", _stable_since=None,
        _align_frames=0, _align_since=None,
        _bridge=bus, MODE_TUNNEL_ALIGN="T_ALIGN",
        _announce=lambda msg: log.append(msg),
        _emit=lambda v, w, now: (v, w),
        _update_height=lambda now: None,
        _show=lambda *a, **k: None,
        _guard_clamped_frames=0,
    )
    return self, bus, log

def run(mod, speeds, dt=1.0/15.0):
    self, bus, log = make_stub(mod, speeds)
    done = None
    t = 0.0
    for sp in speeds:
        bus.speed = sp
        mod.RushFollower._sidewalk_command(self, None, t)
        if self._passed_sidewalk and done is None:
            done = t
        t += dt
    return done, self, log

def noise(seed, n=225):
    rnd = random.Random(seed)
    out = []
    for i in range(n):
        t = i / 15.0
        real = max(0.0, 0.42 * (1.0 - t / 1.2)) if t < 1.2 else 0.0
        v = real + rnd.uniform(-0.07, 0.07)
        if rnd.random() < 0.08:
            v += rnd.choice([-0.14, 0.14])
        out.append(v)
    return out

new = load("/home/orangepi/wheel_robot/src/wheel_robot/scripts/rush.py", "rush_new")
old = load("/tmp/rush_before_standstill_fix.py", "rush_old")

print("=== 真实方法体对照（同一段噪声，15Hz，15 秒）===")
print("%4s  %-26s  %-26s" % ("种子", "旧代码", "新代码"))
ok_old = ok_new = 0
for s in range(1, 13):
    d_old, _, _ = run(old, noise(s))
    # 新代码用了新常量；旧文件里没有这些常量，得临时注入
    for k, v in [("RUSH_STANDSTILL_SPEED", 0.08), ("RUSH_MOVING_RESET_SECONDS", 0.40),
                 ("RUSH_STOP_SPEED_FILTER_ALPHA", 0.25), ("RUSH_SIDEWALK_MAX_SECONDS", 20.0)]:
        setattr(new, k, v)
    d_new, _, _ = run(new, noise(s))
    ok_old += d_old is not None; ok_new += d_new is not None
    print("%4d  %-26s  %-26s" % (s,
        "凑不满，永远停车" if d_old is None else "%.1fs 完成" % d_old,
        "未完成" if d_new is None else "%.1fs 完成" % d_new))
print()
print("旧代码成功 %d/12    新代码成功 %d/12" % (ok_old, ok_new))

print()
print("=== 新代码：真实爬行必须被拒绝 ===")
for creep in (0.10, 0.12, 0.20, 0.30, 0.50):
    for k, v in [("RUSH_STANDSTILL_SPEED", 0.08), ("RUSH_MOVING_RESET_SECONDS", 0.40),
                 ("RUSH_STOP_SPEED_FILTER_ALPHA", 0.25), ("RUSH_SIDEWALK_MAX_SECONDS", 20.0)]:
        setattr(new, k, v)
    d, _, _ = run(new, [creep] * 225)
    print("  持续 %.2f m/s -> %s" % (creep, "错误地判定为停够！" if d is not None else "正确拒绝"))

print()
print("=== 新代码：先动后停（出弯滑行尾段）必须能完成 ===")
seq = [0.40, 0.35, 0.30, 0.25, 0.20, 0.15, 0.10, 0.05] + noise(99, 200)
for k, v in [("RUSH_STANDSTILL_SPEED", 0.08), ("RUSH_MOVING_RESET_SECONDS", 0.40),
             ("RUSH_STOP_SPEED_FILTER_ALPHA", 0.25), ("RUSH_SIDEWALK_MAX_SECONDS", 20.0)]:
    setattr(new, k, v)
d, self, log = run(new, seq)
print("  -> %s" % ("未完成" if d is None else "%.1fs 完成" % d))
print("  清零告警：%d 次" % sum(1 for m in log if "清零" in m))
