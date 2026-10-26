#!/usr/bin/env python3
"""G9 (RT6b): controlled lowering at exit, and the temperature stop. DOMAIN 1, LOOPBACK ONLY.

Every launcher here is `--sim --network-interface lo`; the fixtures are on domain 1.

  (A) end to end. fake_lowstate --follow-cmd (the arm tracks its command) starting at
      q = 0.4 rad, arm temperature rising 0.5 C/s; the launcher in tmux, driven by the
      fake headset; the RT6 arm watcher on domain 1 sends [q] to that tmux pane.
      Expected: WARN at 55 C, [q] at 60 C (2 s), then the launcher lowers the arms to
      [RT7: the watcher's housing [0] rule, WARN 75 C and [q] at 80 C; the fake starts at
      60 C so the timing is unchanged]
      the pose measured at start (0.4) along a planned trajectory at XR_LOWER_SPEED,
      ramps every kp to 0 over XR_RELEASE_T, and exits 0. Nothing published after.
  (B) blocked arm. fake_lowstate at a fixed q = 0 (does not follow), XR_REST_POSE all
      0.4. [q] with no tracking. The command must never lead the measured pose by more
      than max_err (0.15 rad), and the release must still complete.

Run:  python tools/overnight/test_g9_lower_release.py
"""
import os, re, signal, subprocess, sys, tempfile, threading, time, uuid
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools")]
PY = sys.executable
WATCH = Path("/home/unitree/panthera/remote-teleop-20261012/rt6/tools/arm_watch.py")
SPEED, RELEASE_T, HOLD_T, MAX_ERR = 0.3, 1.5, 0.5, 0.15
results = []


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


from fake_xr_client import FakeXR                       # noqa: E402
from tools import port_guard                            # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: E402
ChannelFactoryInitialize(1, "lo")

cmd, state = [], []
def on_cmd(m):
    cmd.append((time.monotonic(), np.array([m.motor_cmd[i].q for i in range(15, 29)]),
                float(m.motor_cmd[15].kp), float(m.motor_cmd[0].kp), float(m.motor_cmd[15].kd)))
def on_state(m):
    state.append((time.monotonic(), np.array([m.motor_state[i].q for i in range(15, 29)])))
_s1 = ChannelSubscriber("rt/lowcmd", LowCmd_); _s1.Init(on_cmd, 10)
_s2 = ChannelSubscriber("rt/lowstate", LowState_); _s2.Init(on_state, 10)

ENV = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
           XR_ARM_VEL_LIMIT="1", XR_LOWER_SPEED=str(SPEED), XR_RELEASE_T=str(RELEASE_T))


def fixture(args):
    return subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo",
                             "--rate", "500"] + args, env=ENV, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    except Exception:
        try: os.killpg(p.pid, signal.SIGKILL)
        except Exception: pass


def wait_file(path, pattern, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            if re.search(pattern, Path(path).read_text(errors="replace")):
                return True
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    return False


def launcher_left():
    # the Python process only: the tmux wrapper `bash -c '...; sleep 20'` carries the same text
    out = subprocess.run(["ps", "-eo", "args="], capture_output=True, text=True).stdout
    return [l for l in out.splitlines()
            if l.split()[:1] == [PY] and "teleop_hand_and_arm.py" in l and "--sim" in l]


tmp = Path(tempfile.mkdtemp(prefix="g9_"))
try:
    # ================================================================== (A)
    print("=== (A) temperature stop -> [q] -> controlled lowering -> release -> exit ===", flush=True)
    fx = fixture(["--follow-cmd", "--q-arms", "0.4", "--temp-arms", "60", "--temp-rise", "0.5"])
    t_fx = time.monotonic()
    time.sleep(3)
    pane = tmp / "pane_a.log"
    lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
            f"echo \"[g9] launcher exited with $?\"; sleep 20")
    env_str = " ".join(f"{k}={ENV[k]}" for k in ("PYTHONUNBUFFERED", "PYTHONNOUSERSITE", "PYTHONDONTWRITEBYTECODE",
                                                 "XR_ARM_VEL_LIMIT", "XR_LOWER_SPEED", "XR_RELEASE_T"))
    subprocess.run(["tmux", "new", "-d", "-s", "g9", "-x", "200", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
    subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g9", f"cat >> {pane}"], check=True)
    xr = FakeXR().start()
    watch_csv = tmp / "watch.csv"
    watch = subprocess.Popen([PY, "-I", "-B", str(WATCH), str(watch_csv), "g9", "1", "lo"], env=ENV,
                             stdout=open(tmp / "watch.out", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    up = wait_file(pane, r"attach check", 120)
    record("A: launcher up in tmux on fakes", up)
    lw = re.search(r"\[lower\] at exit: [^\n]*", pane.read_text(errors="replace")) if up else None
    record("A: the lowering plan is announced at start", bool(lw), lw.group(0)[:110] if lw else "no line")
    time.sleep(2)
    subprocess.run(["tmux", "send-keys", "-t", "g9", "r"])
    record("A: [r] -> tracking", wait_file(pane, r"\[link\] tracking", 10))
    warned = wait_file(tmp / "watch.out", r"WARN temp housing \[0\] 7[5-9]", 60)
    t_warn = time.monotonic() - t_fx
    record("A: watcher WARNs at 75 C (housing [0])", warned, f"at {t_warn:.0f} s (fake: 60 C + 0.5 C/s)")
    stopped = wait_file(tmp / "watch.out", r"STOP temp housing \[0\] 8\d C .* sent q", 40)
    t_q = time.monotonic()
    record("A: watcher sends [q] at 80 C (housing [0], 2 consecutive s)", stopped,
           (tmp / "watch.out").read_text(errors="replace").strip().splitlines()[-1][:100] if stopped else "no STOP line")
    done = wait_file(pane, r"\[g9\] launcher exited with", 40)
    t_exit = time.monotonic()
    text = pane.read_text(errors="replace")
    rc = re.search(r"launcher exited with (-?\d+)", text)
    record("A: launcher exits 0", rc is not None and rc.group(1) == "0", rc.group(0) if rc else "no exit line")
    record("A: [key] q received (sent by the watcher)", "[key] q received" in text)
    ld = re.search(r"\[lower\] (done|TIMED OUT) in ([\d.]+) s", text)
    record("A: lowering completed", bool(ld) and ld.group(1) == "done", ld.group(0) if ld else "no [lower] result")
    xr.stop()
    time.sleep(3)
    seg = [c for c in cmd if c[0] >= t_q]
    if seg:
        # the lowering window: from the first sample whose kp is still full after [q] to the release start
        q_arr = np.array([c[1] for c in seg]); t_arr = np.array([c[0] for c in seg])
        kp15 = np.array([c[2] for c in seg]); kp0 = np.array([c[3] for c in seg]); kd15 = np.array([c[4] for c in seg])
        final = q_arr[-1]
        record("A: arms end at the pose measured at start (0.4 rad, all 14 joints)",
               float(np.max(np.abs(final - 0.4))) < 2e-3, f"max |q - 0.4| {float(np.max(np.abs(final - 0.4))):.4f}")
        moving = [k for k in range(1, len(q_arr)) if np.max(np.abs(q_arr[k] - q_arr[k - 1])) > 1e-6]
        if len(moving) > 2:
            a, b = moving[0] - 1, moving[-1]
            dq = np.max(np.abs(q_arr[b] - q_arr[a])); dt = t_arr[b] - t_arr[a]
            peak = max(np.max(np.abs(q_arr[k] - q_arr[k - 1])) / max(t_arr[k] - t_arr[k - 1], 1e-3) for k in moving)
            record("A: planned lowering speed about XR_LOWER_SPEED", dt > 0 and dq / dt <= SPEED * 1.25,
                   f"{dq:.3f} rad in {dt:.2f} s = {dq / dt:.3f} rad/s (limit {SPEED})")
            gap = dq
            record("A: ...and it closed a real gap (> 0.1 rad)", gap > 0.1, f"{gap:.3f} rad")
        else:
            record("A: lowering motion seen on rt/lowcmd", False, "no motion after [q]")
        record("A: arm kp ramps to 0", kp15[-1] == 0.0 and kp15.max() > 0, f"kp15 {kp15.max():.0f} -> {kp15[-1]:.1f}")
        record("A: leg kp ramps to 0 too", kp0[-1] == 0.0 and kp0.max() > 0, f"kp0 {kp0.max():.0f} -> {kp0[-1]:.1f}")
        rel = kp15[kp15 < kp15.max()]
        record("A: kp never rises during the release", bool(len(rel)) and np.all(np.diff(rel) <= 1e-9))
        record("A: damping kd kept", float(kd15[-1]) > 0, f"kd15 {kd15[-1]:.1f}")
    else:
        record("A: commands captured after [q]", False)
    after = [c for c in cmd if c[0] > t_exit + 1.0]
    record("A: nothing published 1 s after exit", not after, f"{len(after)} samples")
    record("A: no launcher process left", not launcher_left())
    subprocess.run(["tmux", "kill-session", "-t", "g9"], capture_output=True)
    stop(watch); stop(fx)
    time.sleep(2)

    # ================================================================== (B)
    print("\n=== (B) blocked arm: command bounded to max_err from the measured pose ===", flush=True)
    import yaml
    names = list(yaml.safe_load(open(REPO / "poses/ready.yaml"))["joints"].keys())
    rest_yaml = tmp / "rest_04.yaml"
    yaml.safe_dump({"name": "g9_rest", "robot": "G1_29", "units": "radians",
                    "joints": {n: 0.4 for n in names}}, open(rest_yaml, "w"))
    fx = fixture(["--q-arms", "0.0"])
    time.sleep(3)
    cmd.clear(); state.clear()
    log_b = open(tmp / "launcher_b.log", "w")
    envb = dict(ENV, XR_REST_POSE=str(rest_yaml))
    L = subprocess.Popen([PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo", "--ipc"],
                         cwd=str(REPO / "teleop"), env=envb, stdin=subprocess.DEVNULL, stdout=log_b,
                         stderr=subprocess.STDOUT, start_new_session=True)
    ok = wait_file(tmp / "launcher_b.log", r"Press \[r\]", 120)
    record("B: launcher up", ok)
    import zmq
    c = zmq.Context(); s = c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 4000); s.setsockopt(zmq.LINGER, 0)
    s.connect("ipc://@xr_teleoperate_data.ipc"); t_qb = time.monotonic()
    s.send_json({"reqid": str(uuid.uuid4()), "cmd": "CMD_STOP"})
    try: s.recv_json()
    except Exception: pass
    s.close(); c.term()
    try:
        rcb = L.wait(timeout=40)
    except subprocess.TimeoutExpired:
        rcb = None; os.killpg(L.pid, signal.SIGKILL)
    t_eb = time.monotonic()
    tb = (tmp / "launcher_b.log").read_text(errors="replace")
    record("B: exit 0", rcb == 0, f"exit={rcb} after {t_eb - t_qb:.1f} s")
    record("B: the rest pose came from XR_REST_POSE", "XR_REST_POSE yaml" in tb)
    segb = [x for x in cmd if t_qb <= x[0] <= t_eb]
    if segb:
        lead = max(float(np.max(np.abs(x[1] - 0.0))) for x in segb)
        record("B: command never leads the (blocked, q = 0) arm by more than max_err",
               lead <= MAX_ERR + 1e-6, f"max lead {lead:.4f} rad (max_err {MAX_ERR})")
        record("B: kp still released to 0", segb[-1][2] == 0.0 and segb[-1][3] == 0.0,
               f"kp15 {segb[-1][2]:.1f} kp0 {segb[-1][3]:.1f}")
    else:
        record("B: commands captured", False)
    time.sleep(2)
    record("B: no launcher process left", not launcher_left())
    stop(fx)
    record("8012 free at the end", port_guard.port_is_free(8012))
finally:
    subprocess.run(["tmux", "kill-session", "-t", "g9"], capture_output=True)

print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, d in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {d}" if d else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed", flush=True)
from tools._procs import reap_child_processes  # noqa: E402
reap_child_processes()
os._exit(1 if n_fail else 0)
