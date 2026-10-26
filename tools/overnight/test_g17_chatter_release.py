#!/usr/bin/env python3
"""G17 (RT7): chatter detector and release at once. DOMAIN 1, LOOPBACK ONLY.

After the 23 Oct 20:16 lowering, where all six wrist motors chattered at full torque with
raised kd, the owner's rule: on chatter, or any stop during the lowering, release at once
(zero torque); never keep gains that chattered. Two parts:
  * rt6/tools/arm_watch.py flags CHATTER (one motor: >= 100 dq sign flips and >= 5 Nm
    tau_est swing in one second), writes ARM_WATCH_RELEASE_FILE once and sends [q];
  * the launcher's arm controller polls XR_RELEASE_FILE (only if written after it started)
    and goes to kp = kd = 0, tau = 0 on every motor, latched; a [q] during the lowering does
    the same.

  (A) watcher alone, fake chatter on motor 20 (dq +-1.5 rad/s, tau +-20 Nm every sample):
      CHATTER within 2 s, release file written, one [q]; control: the simulated arm at rest
      with 0.015 rad/s dq noise for 8 s: no CHATTER.
  (B) launcher alone: (1) release file during FOLLOW: zero torque within 0.1 s on every
      motor, stays zero, no lowering at [q], exit 0; (2) [q], then a second [q] 1.5 s into
      the lowering: zero torque within 0.1 s, exit 0; (3) a release file older than the
      launch is ignored.
  (C) end to end: fake chatter during FOLLOW, the watcher writes the file, the launcher goes
      to zero torque within 3 s of the chatter's start and exits 0 on the watcher's [q].
  Plus: the lowering keeps the following kd (no gain raised) and its command moves at most
      0.2 rad/s.

Run:  python tools/overnight/test_g17_chatter_release.py
"""
import os, re, signal, subprocess, sys, tempfile, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools")]
PY = sys.executable
WATCH = Path("/home/unitree/panthera/remote-teleop-20261012/rt6/tools/arm_watch.py")
results = []


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


def summary_and_exit():
    print("\n=== summary ===")
    n_fail = sum(1 for _, ok, _ in results if not ok)
    for name, ok, d in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {d}" if d else ""))
    print(f"\n{len(results) - n_fail}/{len(results)} passed", flush=True)
    from tools._procs import reap_child_processes
    reap_child_processes()
    os._exit(1 if n_fail else 0)


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


def wait_file(path, pattern, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            m = re.search(pattern, Path(path).read_text(errors="replace"))
            if m:
                return m
        except FileNotFoundError:
            pass
        time.sleep(0.1)
    return None


from fake_xr_client import FakeXR                                     # noqa: E402
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_            # noqa: E402
ChannelFactoryInitialize(1, "lo")
cmd = []
def on_cmd(m):
    mc = m.motor_cmd
    cmd.append((time.monotonic(), max(mc[i].kp for i in range(29)), max(mc[i].kd for i in range(29)),
                max(abs(mc[i].tau) for i in range(29)), np.array([mc[i].q for i in range(15, 29)]),
                np.array([mc[i].kd for i in range(15, 29)])))
_s = ChannelSubscriber("rt/lowcmd", LowCmd_); _s.Init(on_cmd, 10)
tmp = Path(tempfile.mkdtemp(prefix="g17_"))
BASE = {k: v for k, v in os.environ.items() if not k.startswith(("ARM_WATCH_", "XR_RELEASE"))}
BASE.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1")


def fake(args):
    return subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo", "--rate", "500"]
                            + args, env=BASE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def watcher(tag, pane, env_extra):
    out_csv, out_txt = tmp / f"watch_{tag}.csv", tmp / f"watch_{tag}.out"
    w = subprocess.Popen([PY, "-I", "-B", str(WATCH), str(out_csv), pane, "1", "lo"], env=dict(BASE, **env_extra),
                         stdout=open(out_txt, "w"), stderr=subprocess.STDOUT, start_new_session=True)
    return w, out_csv, out_txt


def dummy_pane(keys):
    subprocess.run(["tmux", "kill-session", "-t", "g17w"], capture_output=True)
    subprocess.run(["tmux", "new", "-d", "-s", "g17w", "-x", "80", "-y", "10",
                    f"bash -c 'stty -icanon -echo min 1; cat > {keys}'"], check=True)


# ================================================================= (A) watcher alone
print("=== (A) watcher: chatter detector ===", flush=True)
keys = tmp / "keys_a.txt"; keys.write_text("")
dummy_pane(keys)
flag = tmp / "chatter_a.on"
rel = tmp / "release_a"
fx = fake(["--q-arms", "0.0", "--chatter", "20", "--chatter-file", str(flag)])
time.sleep(1.5)
w, wcsv, wout = watcher("a", "g17w", {"ARM_WATCH_RELEASE_FILE": str(rel)})
time.sleep(3.0)
t_on = time.monotonic(); flag.write_text("on")
got = wait_file(wout, r"CHATTER motor 20", 6)
t_det = time.monotonic()
time.sleep(2.0)
stop(w); stop(fx); flag.unlink(missing_ok=True)
subprocess.run(["tmux", "kill-session", "-t", "g17w"], capture_output=True)
txt = wout.read_text(errors="replace")
record("A: thresholds line names the chatter rule and the release file", "chatter: >= 100 dq sign flips and >= 5 Nm" in txt
       and "release file set" in txt)
record("A: CHATTER on motor 20 within 2.5 s of its start", bool(got) and t_det - t_on <= 2.5, f"{t_det - t_on:.1f} s; {got.group(0) if got else 'none'}")
record("A: release file written with the reason", rel.exists() and "CHATTER motor 20" in rel.read_text(), rel.read_text().strip()[:90] if rel.exists() else "missing")
record("A: exactly one [q] to the pane", keys.read_text().count("q") == 1, f"keys {keys.read_text()!r}")
rows = [l.split(",") for l in wcsv.read_text().splitlines()[1:]]
record("A: CSV has the chatter columns", "chat_flips_max" in wcsv.read_text().splitlines()[0])

rel_c = tmp / "release_ctrl"
fx = fake(["--physics", "--pose", str(REPO / "poses/low.yaml"), "--dq-noise", "0.015"])
time.sleep(1.5)
w, wcsv, wout = watcher("ctrl", "g17none", {"ARM_WATCH_RELEASE_FILE": str(rel_c)})
time.sleep(8.0)
stop(w); stop(fx)
txt = wout.read_text(errors="replace")
hdr = wcsv.read_text().splitlines()[0].split(",")
fl = [int(dict(zip(hdr, l.split(",")))["chat_flips_max"]) for l in wcsv.read_text().splitlines()[1:]]
record("A: control, simulated arm at rest with 0.015 rad/s dq noise, 8 s: no CHATTER, no release file",
       "CHATTER" not in txt and not rel_c.exists(), f"dq sign flips per second max {max(fl) if fl else '-'} (threshold 100)")

# ================================================================= (B), (C) launcher
ENVL = {"XR_ARM_VEL_LIMIT": "1", "XR_SIM_VEL_CLIP": "1", "XR_SHOULDER_TAU_CAP": "0"}


def launch(tag, rel_file, fake_args, watch_env=None, pre=None):
    fx = fake(["--follow-cmd", "--pose", str(REPO / "poses/low.yaml")] + fake_args)
    time.sleep(2)
    if pre:
        pre()
    pane = tmp / f"pane_{tag}.log"
    env = dict(BASE, **ENVL, XR_RELEASE_FILE=str(rel_file))
    keys_ = ["PYTHONUNBUFFERED", "PYTHONNOUSERSITE", "PYTHONDONTWRITEBYTECODE", "XR_RELEASE_FILE"] + list(ENVL)
    env_str = " ".join(f"{k}={env[k]}" for k in keys_)
    lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
            f"echo \"[g17] launcher exited with $?\"; sleep 15")
    subprocess.run(["tmux", "kill-session", "-t", "g17"], capture_output=True)
    subprocess.run(["tmux", "new", "-d", "-s", "g17", "-x", "200", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
    subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g17", f"cat >> {pane}"], check=True)
    w = None
    if watch_env is not None:
        w = watcher(tag, "g17", watch_env)[0]
    xr = FakeXR(radius=0.01).start()
    up = wait_file(pane, r"attach check", 120)
    time.sleep(2)
    subprocess.run(["tmux", "send-keys", "-t", "g17", "r"])
    tr = wait_file(pane, r"\[link\] tracking", 15)
    return dict(fx=fx, pane=pane, xr=xr, up=bool(up), tr=bool(tr), w=w)


def finish(L, send_q=True):
    if send_q:
        subprocess.run(["tmux", "send-keys", "-t", "g17", "q"])
    m = wait_file(L["pane"], r"\[g17\] launcher exited with (-?\d+)", 90)
    L["xr"].stop()
    subprocess.run(["tmux", "kill-session", "-t", "g17"], capture_output=True)
    if L["w"] is not None:
        stop(L["w"])
    stop(L["fx"])
    time.sleep(2)
    return m.group(1) if m else None


def zero_after(t0, dt=0.1):
    """(first all-zero sample time - t0, any sample after it with kp/kd/tau > 0, samples after)"""
    seg = [c for c in cmd if c[0] >= t0]
    z = next((c for c in seg if c[1] == 0 and c[2] == 0 and c[3] == 0), None)
    if z is None:
        return None, None, 0
    after = [c for c in seg if c[0] > z[0]]
    bad = any(c[1] > 0 or c[2] > 0 or c[3] > 0 for c in after)
    return z[0] - t0, bad, len(after)


print("\n=== (B1) release file during FOLLOW ===", flush=True)
relb = tmp / "release_b1"
L = launch("b1", relb, [])
record("B1: launcher up, release line, tracking", L["up"] and L["tr"] and "release at once (zero torque) on chatter"
       in L["pane"].read_text(errors="replace"))
lw = re.search(r"\[lower\] at exit: [^\n]*", L["pane"].read_text(errors="replace"))
record("B1: the lowering keeps the following kd and caps the command at 0.2 rad/s",
       bool(lw) and "kd as while following (no gain raised)" in lw.group(0) and "command rate limited to 0.2 rad/s" in lw.group(0),
       lw.group(0)[-160:] if lw else "no line")
time.sleep(3)
seg = [c for c in cmd if c[0] >= time.monotonic() - 1.0]
record("B1: before the release: gains on (kp > 0)", seg and min(c[1] for c in seg) > 0)
t_rel = time.monotonic(); relb.write_text("test: chatter")
time.sleep(2.0)
lat, bad, n_after = zero_after(t_rel)
record("B1: zero torque on every motor within 0.1 s of the release file", lat is not None and lat <= 0.1,
       f"{lat if lat is None else round(lat, 3)} s")
record("B1: ...and it stays zero (latched)", lat is not None and not bad and n_after > 200, f"{n_after} samples after")
t_q = time.monotonic()
rc = finish(L)
text = L["pane"].read_text(errors="replace")
record("B1: [q] after the release: no lowering, launcher says released, exit 0",
       "RELEASED AT ONCE" in text and "[lower] lowering the arms" not in text and rc == "0", f"exit {rc}")
after = [c for c in cmd if c[0] >= t_q]
record("B1: nothing with gains after the [q]", not any(c[1] > 0 or c[2] > 0 for c in after), f"{len(after)} samples")

print("\n=== (B2) a second [q] during the lowering ===", flush=True)
L = launch("b2", tmp / "release_b2", [])
record("B2: launcher up, tracking", L["up"] and L["tr"])
time.sleep(4)
subprocess.run(["tmux", "send-keys", "-t", "g17", "q"])
low = wait_file(L["pane"], r"\[lower\] lowering the arms", 10)
t_low = time.monotonic()
time.sleep(1.5)
seg = [c for c in cmd if t_low <= c[0]]
kd_lw = max((float(c[5].max()) for c in seg), default=0.0)
qs = np.array([c[4] for c in seg]); ts = np.array([c[0] for c in seg])
v = (np.max(np.abs(np.diff(qs, axis=0)), axis=1) / np.maximum(np.diff(ts), 1e-3)) if len(seg) > 10 else np.zeros(1)
record("B2: during the lowering no arm kd is raised (max kd 3)", low and kd_lw <= 3.0 + 1e-6, f"max arm kd {kd_lw:g}")
record("B2: the lowering command moves at most 0.2 rad/s (median of 10 samples)",
       len(v) > 10 and float(np.percentile(v, 90)) <= 0.2 * 1.3, f"90th percentile {float(np.percentile(v, 90)):.3f} rad/s")
t_q2 = time.monotonic()
subprocess.run(["tmux", "send-keys", "-t", "g17", "q"])
time.sleep(1.5)
lat, bad, n_after = zero_after(t_q2)
record("B2: second [q] during the lowering: zero torque within 0.15 s, latched", lat is not None and lat <= 0.15 and not bad,
       f"{lat if lat is None else round(lat, 3)} s, {n_after} samples after")
rc = finish(L, send_q=False)
text = L["pane"].read_text(errors="replace")
record("B2: launcher says released during the lowering, exit 0",
       "[q] during the lowering: RELEASED AT ONCE" in text and rc == "0", f"exit {rc}")

print("\n=== (B3) a stale release file is ignored ===", flush=True)
rel3 = tmp / "release_b3"
rel3.write_text("stale")
os.utime(rel3, (time.time() - 600, time.time() - 600))
L = launch("b3", rel3, [])
time.sleep(4)
seg = [c for c in cmd if c[0] >= time.monotonic() - 2.0]
record("B3: release file older than the launch: gains stay on", L["tr"] and seg and min(c[1] for c in seg) > 0)
rc = finish(L)
record("B3: exit 0", rc == "0", f"exit {rc}")

print("\n=== (C) end to end: fake chatter during FOLLOW ===", flush=True)
flag = tmp / "chatter_c.on"
relc = tmp / "release_c"
L = launch("c", relc, ["--chatter", "20", "26", "--chatter-file", str(flag)], watch_env={"ARM_WATCH_RELEASE_FILE": str(relc)})
record("C: launcher up, tracking", L["up"] and L["tr"])
time.sleep(4)
t_on = time.monotonic(); flag.write_text("on")
time.sleep(4)
lat, bad, n_after = zero_after(t_on)
record("C: zero torque within 3 s of the chatter's start (watcher, release file, controller)",
       lat is not None and lat <= 3.0 and not bad, f"{lat if lat is None else round(lat, 2)} s")
rc = finish(L, send_q=False)
flag.unlink(missing_ok=True)
text = L["pane"].read_text(errors="replace")
record("C: the watcher's [q] ends the launcher, no lowering, exit 0",
       "[key] q received" in text and "[lower] lowering the arms" not in text and rc == "0", f"exit {rc}")
record("8012 free at the end", port_guard.port_is_free(8012))
summary_and_exit()
