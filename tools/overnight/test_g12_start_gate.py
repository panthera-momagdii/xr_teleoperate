#!/usr/bin/env python3
"""G12 (RT6f): arm start gate and the arm watcher's angle columns. CPU and DOMAIN 1 / LOOPBACK ONLY.

  (A) CPU. teleop/robot_control/start_gate.py on its own: URDF limits, the 0.05 rad
      margin both sides of the edge, the "arms were moved" check at 0.2 rad, the env
      knobs, the released-pose file. Tonight's RT6e pre-check pose (22 Oct 02:01, left
      shoulder yaw 0.014 rad from its limit after the robot was handled) must trigger the
      refusal AND, against the last logged released pose, the moved warning.
  (B) End to end on fakes, launcher --sim --ipc on lo:
      B1 the fake reports tonight's pose: exit 7, the joint and distance printed, the
         moved warning printed, NOTHING published on rt/lowcmd, no debug-mode entry, no
         arm controller, the released-pose file untouched.
      B2 the fake at the low pose, file = that pose: no warning, the launcher starts,
         [q] lowers and exits 0, and the file is rewritten with the released pose.
      B3 the fake at the low pose with the left elbow 0.3 rad off the file: the warning
         names left_elbow_joint, and the launcher still starts (a warning, not a refusal).
      B4 defence in depth: G1_29_ArmController itself, with REST_GUARD set, refuses to
         record tonight's pose as its rest pose, before its publish thread starts.
  (C) rt6/tools/arm_watch.py on domain 1 against a fake at the low pose: the eight
      shoulder and elbow angle columns are present and read the pose.

Run:  python tools/overnight/test_g12_start_gate.py [--cpu-only]
"""
import os, re, signal, subprocess, sys, tempfile, time, uuid
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools"), str(REPO / "teleop")]
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


from teleop.robot_control import start_gate as g             # noqa: E402
from teleop.robot_control.start_pose import ARM_JOINT_NAMES, load_pose  # noqa: E402

# rt6/out/rt6e_precheck/arm_pose.json (22 Oct 02:01, 1825 samples, arms limp)
Q_TONIGHT = np.array([0.0059, 0.0588, 2.6036, 1.476, -1.653, 0.0403, -0.0177,
                      -0.0164, 0.0016, -1.145, 1.3896, 1.5396, 0.1216, -0.0479])
# rt6/out/arm_pose_start.json (21 Oct 23:34, before RT6): the last logged released pose.
# RT6c's release pose was never logged, which is what XR_LAST_POSE_FILE fixes from now on.
Q_LOGGED = np.array([0.1163, 0.0381, 1.2945, 1.4639, 1.3122, 0.0982, 0.145,
                     0.1816, -0.0734, 0.4694, 1.4292, 0.3173, 0.1304, -0.1723])
Q_LOW = load_pose(str(REPO / "poses" / "low.yaml"))
tmp = Path(tempfile.mkdtemp(prefix="g12_"))

# ====================================================================== (A) CPU
print("=== (A) CPU: start_gate on its own ===", flush=True)
lo, hi = g.urdf_arm_limits()
k_lsy = ARM_JOINT_NAMES.index("left_shoulder_yaw_joint")
record("A: URDF limits for all 14 arm joints", len(lo) == 14 and abs(hi[k_lsy] - 2.618) < 1e-3,
       f"left shoulder yaw {lo[k_lsy]:+.3f} to {hi[k_lsy]:+.3f} rad")
viol = g.limit_violations(Q_TONIGHT, lo, hi, 0.05)
txt = g.describe_violations(viol, 0.05) if viol else ""
record("A: tonight's pose is refused", bool(viol))
record("A: ...naming the joint, the angle, the limit and the distance",
       "left_shoulder_yaw_joint at +2.604 rad is 0.014 rad from its upper limit +2.618 rad" in txt,
       txt.splitlines()[1].strip() if viol else "")
record("A: ...and only that joint", len(viol) == 1)
mv = g.moved_joints(Q_TONIGHT, Q_LOGGED, 0.2)
mtxt = g.describe_moved(mv, 0.2, "21 Oct 23:34")
record("A: tonight's pose against the last logged released pose: 'arms were moved'",
       "arms were moved since the last session" in mtxt and len(mv) == 4,
       "; ".join(f"{n} {d:+.3f}" for n, d in mv))
record("A: the low pose is clear of every limit", not g.limit_violations(Q_LOW, lo, hi, 0.05))
q = Q_LOW.copy(); q[k_lsy] = hi[k_lsy] - 0.049
record("A: 0.049 rad from a limit refuses", bool(g.limit_violations(q, lo, hi, 0.05)))
q[k_lsy] = hi[k_lsy] - 0.051
record("A: 0.051 rad from a limit passes", not g.limit_violations(q, lo, hi, 0.05))
q[k_lsy] = lo[k_lsy] - 0.01
v2 = g.limit_violations(q, lo, hi, 0.05)
record("A: past a limit says PAST and names the lower limit",
       bool(v2) and "PAST its lower limit" in g.describe_violations(v2, 0.05))
q = Q_LOW.copy(); q[3] += 0.19
record("A: 0.19 rad off the released pose: no warning", not g.moved_joints(q, Q_LOW, 0.2))
q[3] = Q_LOW[3] + 0.21
record("A: 0.21 rad off: warning names the joint", [n for n, _ in g.moved_joints(q, Q_LOW, 0.2)] == ["left_elbow_joint"])
for k in ("XR_GATE_LIMIT_MARGIN", "XR_GATE_MOVED_TOL", "XR_LAST_POSE_FILE"):
    os.environ.pop(k, None)
record("A: defaults 0.05 / 0.2 / no file", g.settings_from_env() == (0.05, 0.2, None))
bad = []
for k, v in (("XR_GATE_LIMIT_MARGIN", "-0.1"), ("XR_GATE_MOVED_TOL", "abc")):
    os.environ[k] = v
    try:
        g.settings_from_env(); bad.append(k)
    except ValueError:
        pass
    os.environ.pop(k, None)
record("A: bad knob values refused", not bad, f"accepted {bad}" if bad else "")
pf = tmp / "last.yaml"
g.save_last(str(pf), Q_LOW, note="g12")
back = g.load_last(str(pf))
record("A: released-pose file round trip", back is not None and np.allclose(back[0], Q_LOW, atol=1e-5),
       f"released_at {back[1] if back else '-'}")
record("A: the file is a valid start pose too", np.allclose(load_pose(str(pf)), Q_LOW, atol=1e-5))
refuse, notes = g.check(Q_TONIGHT, lo, hi, 0.05, 0.2, None)
record("A: no file configured: the moved check says it is off", bool(refuse) and any("moved check off" in n for n in notes))

if "--cpu-only" in sys.argv:
    summary_and_exit()

# ====================================================================== (B) fakes
print("\n=== (B) end to end on fakes (domain 1, lo, --sim --ipc) ===", flush=True)
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_            # noqa: E402
ChannelFactoryInitialize(1, "lo")
cmdlog = []
_s = ChannelSubscriber("rt/lowcmd", LowCmd_); _s.Init(lambda m: cmdlog.append(time.monotonic()), 10)
BASE = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
            XR_ARM_VEL_LIMIT="1")


def pose_yaml(q, name):
    import yaml
    p = tmp / f"{name}.yaml"
    yaml.safe_dump({"name": name, "robot": "G1_29", "units": "radians",
                    "joints": {n: float(v) for n, v in zip(ARM_JOINT_NAMES, q)}}, open(p, "w"))
    return p


def fixture(q, name):
    return subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo",
                             "--rate", "500", "--pose", str(pose_yaml(q, name))], env=BASE,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


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
            m = re.search(pattern, Path(path).read_text(errors="replace"))
            if m:
                return m
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    return None


def launcher(tag, last_file):
    log = tmp / f"launcher_{tag}.log"
    env = dict(BASE, XR_LAST_POSE_FILE=str(last_file))
    p = subprocess.Popen([PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo", "--ipc"],
                         cwd=str(REPO / "teleop"), env=env, stdin=subprocess.DEVNULL,
                         stdout=open(log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
    return p, log


def ipc_stop():
    import zmq
    c = zmq.Context(); s = c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 4000); s.setsockopt(zmq.LINGER, 0)
    s.connect("ipc://@xr_teleoperate_data.ipc")
    s.send_json({"reqid": str(uuid.uuid4()), "cmd": "CMD_STOP"})
    try: s.recv_json()
    except Exception: pass
    s.close(); c.term()


try:
    # ---- B1: tonight's pose
    last = tmp / "last_b1.yaml"
    g.save_last(str(last), Q_LOGGED, note="g12 B1: the 21 Oct 23:34 logged pose")
    before = last.read_bytes()
    fx = fixture(Q_TONIGHT, "tonight")
    time.sleep(2)
    cmdlog.clear(); t0 = time.monotonic()
    L, log = launcher("b1", last)
    try:
        rc = L.wait(timeout=60)
    except subprocess.TimeoutExpired:
        rc = None; os.killpg(L.pid, signal.SIGKILL)
    txt = log.read_text(errors="replace")
    record("B1: tonight's pose: launcher refuses with exit 7", rc == 7, f"exit {rc} after {time.monotonic() - t0:.1f} s")
    record("B1: ...names the joint and the distance",
           "left_shoulder_yaw_joint at +2.604 rad is 0.014 rad from its upper limit +2.618 rad" in txt)
    record("B1: ...and prints the moved warning", "arms were moved since the last session" in txt)
    record("B1: nothing published on rt/lowcmd", not cmdlog, f"{len(cmdlog)} samples")
    record("B1: no debug-mode entry, no arm controller", "Enter debug mode" not in txt
           and "Lock OK" not in txt and "Initialize G1_29_ArmController" not in txt)
    record("B1: the released-pose file untouched", last.read_bytes() == before)
    stop(fx); time.sleep(2)

    # ---- B2: the low pose, file = the low pose; start, [q], file rewritten
    last2 = tmp / "last_b2.yaml"
    g.save_last(str(last2), Q_LOW, note="g12 B2")
    t_saved = g.load_last(str(last2))[1]
    time.sleep(1.1)
    fx = fixture(Q_LOW, "low")
    time.sleep(2)
    L, log = launcher("b2", last2)
    up = wait_file(log, r"Press \[r\]", 120)
    txt = log.read_text(errors="replace")
    record("B2: the low pose starts", bool(up))
    record("B2: no moved warning, 'within 0.2 rad of the last released pose'",
           "within 0.2 rad of the last released pose" in txt and "arms were moved" not in txt)
    record("B2: the clear-of-limits line", "[gate] arm joints clear of their limits" in txt)
    ipc_stop()
    try:
        rc = L.wait(timeout=60)
    except subprocess.TimeoutExpired:
        rc = None; os.killpg(L.pid, signal.SIGKILL)
    txt = log.read_text(errors="replace")
    after = g.load_last(str(last2))
    record("B2: [q] -> exit 0 and 'released pose saved'", rc == 0 and "[gate] released pose saved" in txt, f"exit {rc}")
    record("B2: the file now holds the released pose, newer", after is not None and after[1] != t_saved
           and np.allclose(after[0], Q_LOW, atol=1e-3), f"released_at {after[1] if after else '-'}")
    stop(fx); time.sleep(2)

    # ---- B3: moved but clear: a warning, not a refusal
    q3 = Q_LOW.copy(); q3[3] += 0.3
    fx = fixture(q3, "moved")
    time.sleep(2)
    L, log = launcher("b3", last2)
    up = wait_file(log, r"Press \[r\]", 120)
    txt = log.read_text(errors="replace")
    record("B3: moved 0.3 rad: warning names left_elbow_joint +0.300", "left_elbow_joint moved +0.300 rad" in txt)
    record("B3: ...and the launcher still starts", bool(up))
    ipc_stop()
    try:
        rc = L.wait(timeout=60)
    except subprocess.TimeoutExpired:
        rc = None; os.killpg(L.pid, signal.SIGKILL)
    record("B3: exit 0", rc == 0, f"exit {rc}")
    stop(fx); time.sleep(2)

    # ---- B4: the controller's own guard (the arms moved between the gate and the controller)
    fx = fixture(Q_TONIGHT, "tonight_b4")
    time.sleep(2)
    from teleop.robot_control.robot_arm import G1_29_ArmController      # noqa: E402
    G1_29_ArmController.REST_GUARD = (lo, hi, 0.05)
    cmdlog.clear()
    err = None
    try:
        G1_29_ArmController(motion_mode=False, simulation_mode=True)
    except g.StartGateError as exc:
        err = str(exc)
    finally:
        G1_29_ArmController.REST_GUARD = None
    time.sleep(1.0)
    record("B4: the arm controller refuses to record tonight's pose as the rest pose",
           err is not None and "left_shoulder_yaw_joint" in err, (err or "no refusal").splitlines()[0][:90])
    record("B4: ...before publishing anything", not cmdlog, f"{len(cmdlog)} samples on rt/lowcmd")
    stop(fx); time.sleep(2)

    # ====================================================================== (C) watcher
    print("\n=== (C) arm watcher angle columns (domain 1, lo) ===", flush=True)
    fx = fixture(Q_LOW, "low_c")
    time.sleep(2)
    csvp = tmp / "watch.csv"
    w = subprocess.Popen([PY, "-I", "-B", str(WATCH), str(csvp), "g12-no-pane", "1", "lo"], env=BASE,
                         stdout=open(tmp / "watch.out", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(5)
    stop(w); stop(fx)
    lines = csvp.read_text().strip().splitlines()
    hdr = lines[0].split(",")
    cols = ["L_sh_pitch_rad", "L_sh_roll_rad", "L_sh_yaw_rad", "L_elbow_rad",
            "R_sh_pitch_rad", "R_sh_roll_rad", "R_sh_yaw_rad", "R_elbow_rad"]
    record("C: header has the 8 shoulder and elbow angle columns", all(c in hdr for c in cols))
    rows = [dict(zip(hdr, l.split(","))) for l in lines[1:]]
    rows = [r for r in rows if r.get("L_sh_pitch_rad") not in (None, "", "nan")]
    want = [Q_LOW[i] for i in (0, 1, 2, 3, 7, 8, 9, 10)]
    ok = bool(rows) and all(abs(float(rows[-1][c]) - v) < 1e-3 for c, v in zip(cols, want))
    record("C: the angles read the fake's pose", ok,
           " ".join(f"{float(rows[-1][c]):+.3f}" for c in cols) if rows else "no rows")
    record("C: older columns unchanged (tracking error, temperature, shoulder torque)",
           hdr[:22] == ("time,lowstate_n,lowcmd_n,track_err_max_rad,track_err_mean_rad,track_err_joint,"
                        "dq_absmax_rad_s,dq_joint,temp_max_C,temp_joint,note,spL_tau_est_Nm,spL_tau_ff_Nm,"
                        "spL_pd_est_Nm,spL_err_rad,spL_temp_C,spR_tau_est_Nm,spR_tau_ff_Nm,spR_pd_est_Nm,"
                        "spR_err_rad,spR_temp_C,spL_kp").split(","))
    record("8012 free at the end", port_guard.port_is_free(8012))
finally:
    pass

summary_and_exit()
