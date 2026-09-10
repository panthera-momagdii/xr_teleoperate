#!/usr/bin/env python3
"""G6: --record works with no --ee (arm-only) and with --ee. DOMAIN 1, LOOPBACK ONLY.

Drives the REAL launcher end to end -- fake rt/lowstate on domain 1, --ipc to send
[r] and [s] without a keyboard -- and then reads the episode off disk and checks its
shapes. A recording path that is only tested by reading the code is not tested.

--ipc uses abstract unix sockets (ipc://@xr_teleoperate_*.ipc), so nothing here binds a
TCP port.

Run:  python tools/overnight/test_g6_record.py
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))
PY = sys.executable

results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def ipc_send(cmd, timeout_ms=4000):
    import zmq
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    try:
        sock.connect("ipc://@xr_teleoperate_data.ipc")
        sock.send_json({"reqid": str(uuid.uuid4()), "cmd": cmd})
        return sock.recv_json()
    except Exception as exc:
        return {"status": "error", "msg": f"{type(exc).__name__}: {exc}"}
    finally:
        sock.close()
        ctx.term()


def kill_group(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except OSError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


def run_session(extra_args, task_dir, seconds=5.0, env_extra=None):
    """Start the launcher with a fake lowstate, record for `seconds`, stop, return
    (launcher output, the episode directory or None)."""
    fake = subprocess.Popen(
        [PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo",
         "--seconds", "120", "--rate", "500", "--q-arms", "0.15"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    env = dict(os.environ)
    env.setdefault("PYTHONUNBUFFERED", "1")
    if env_extra:
        env.update(env_extra)
    with tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False) as fh:
        log_path = fh.name
    proc = None
    try:
        time.sleep(3.0)
        with open(log_path, "wb") as sink:
            proc = subprocess.Popen(
                [PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo",
                 "--ipc", "--headless", "--record",
                 "--task-dir", str(task_dir), "--task-name", "g6",
                 "--frequency", "30"] + extra_args,
                cwd=str(REPO / "teleop"), env=env, stdout=sink,
                stderr=subprocess.STDOUT, start_new_session=True)
            # wait for the [r] prompt
            deadline = time.monotonic() + 60
            ready = False
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    break
                if "Press [r] to start syncing" in Path(log_path).read_text(errors="replace"):
                    ready = True
                    break
                time.sleep(0.5)
            if ready:
                time.sleep(1.0)
                ipc_send("CMD_START")            # [r]
                time.sleep(1.5)
                ipc_send("CMD_RECORD_TOGGLE")    # [s] -> start recording
                time.sleep(seconds)
                ipc_send("CMD_RECORD_TOGGLE")    # [s] -> save
                time.sleep(3.0)
                ipc_send("CMD_STOP")             # [q]
                try:
                    proc.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    kill_group(proc)
            else:
                kill_group(proc)
        out = Path(log_path).read_text(errors="replace")
    finally:
        os.unlink(log_path)
        if proc is not None and proc.poll() is None:
            kill_group(proc)
        fake.terminate()
        try:
            fake.wait(timeout=5)
        except subprocess.TimeoutExpired:
            fake.kill()

    eps = sorted((Path(task_dir) / "g6").glob("episode_*")) if (Path(task_dir) / "g6").exists() else []
    return out, (eps[-1] if eps else None)


def check_episode(tag, ep_dir, expect_ee, expect_ee_dof):
    data_json = ep_dir / "data.json"
    record(f"{tag}: data.json exists", data_json.exists(), str(data_json))
    if not data_json.exists():
        return None
    try:
        doc = json.loads(data_json.read_text())
    except json.JSONDecodeError as exc:
        record(f"{tag}: data.json is valid JSON", False, str(exc))
        return None
    record(f"{tag}: data.json is valid JSON (episode closed properly)", True,
           f"{len(doc.get('data', []))} items")

    info = doc.get("info", {})
    src = info.get("source", {})
    record(f"{tag}: metadata records the end effector",
           src.get("end_effector", {}).get("present") == (expect_ee is not None)
           and src.get("ee") == expect_ee,
           f"source.ee={src.get('ee')!r} present={src.get('end_effector', {}).get('present')}")
    record(f"{tag}: metadata records the arm", src.get("arm") == "G1_29", src.get("arm"))
    jn = info.get("joint_names", {})
    record(f"{tag}: arm joint names are recorded (were empty upstream)",
           len(jn.get("left_arm", [])) == 7 and len(jn.get("right_arm", [])) == 7,
           f"left_arm={len(jn.get('left_arm', []))} right_arm={len(jn.get('right_arm', []))}")
    record(f"{tag}: ee joint names match the end effector",
           len(jn.get("left_ee", [])) == expect_ee_dof,
           f"{len(jn.get('left_ee', []))} names, expected {expect_ee_dof}")

    items = doc.get("data", [])
    record(f"{tag}: recorded a usable number of items", len(items) >= 30,
           f"{len(items)} items at 30 Hz")
    if not items:
        return doc
    it = items[0]
    st, ac = it.get("states", {}), it.get("actions", {})
    record(f"{tag}: left/right arm qpos are 7 long",
           len(st["left_arm"]["qpos"]) == 7 and len(st["right_arm"]["qpos"]) == 7
           and len(ac["left_arm"]["qpos"]) == 7 and len(ac["right_arm"]["qpos"]) == 7,
           f"state {len(st['left_arm']['qpos'])}, action {len(ac['left_arm']['qpos'])}")
    record(f"{tag}: ee qpos length matches the end effector",
           len(st["left_ee"]["qpos"]) == expect_ee_dof
           and len(ac["left_ee"]["qpos"]) == expect_ee_dof,
           f"state {len(st['left_ee']['qpos'])}, action {len(ac['left_ee']['qpos'])}, "
           f"expected {expect_ee_dof}")
    record(f"{tag}: every item has the same shapes",
           all(len(i["states"]["left_arm"]["qpos"]) == 7
               and len(i["states"]["left_ee"]["qpos"]) == expect_ee_dof
               for i in items))
    record(f"{tag}: arm state reflects the fake lowstate (q=0.15)",
           all(abs(v - 0.15) < 1e-6 for v in st["left_arm"]["qpos"]),
           f"{[round(v, 4) for v in st['left_arm']['qpos'][:3]]}")
    return doc


# ============================================================== arm-only
print("\n=== (1) --record with NO --ee (arm-only episodes) ===")
tmp1 = tempfile.mkdtemp(prefix="g6_noee_")
out1, ep1 = run_session([], tmp1, seconds=5.0)
record("arm-only: launcher reached the [r] prompt",
       "Press [r] to start syncing" in out1)
record("arm-only: an episode directory was created", ep1 is not None,
       str(ep1) if ep1 else "none found")
doc1 = check_episode("arm-only", ep1, None, 0) if ep1 else None

# keep this one as the worked example
if ep1 is not None:
    keep = REPO / "logs/overnight/example_episode_arm_only"
    if keep.exists():
        shutil.rmtree(keep)
    shutil.copytree(ep1, keep)
    record("arm-only: example episode kept in logs/overnight/", keep.exists(),
           str(keep.relative_to(REPO)))

# ============================================================== with a hand
print("\n=== (2) --record with --ee inspire_ftp ===")
fake_hand = subprocess.Popen(
    [PY, str(REPO / "tools/fake_inspire_state.py"), "--domain", "1", "--iface", "lo",
     "--angle", "400", "--seconds", "120"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
try:
    time.sleep(2.0)
    tmp2 = tempfile.mkdtemp(prefix="g6_ftp_")
    out2, ep2 = run_session(["--ee", "inspire_ftp"], tmp2, seconds=5.0)
    record("inspire_ftp: launcher reached the [r] prompt",
           "Press [r] to start syncing" in out2,
           "exit 3 (no hand state)" if "REFUSING TO START" in out2 else "")
    record("inspire_ftp: an episode directory was created", ep2 is not None,
           str(ep2) if ep2 else "none found")
    if ep2:
        check_episode("inspire_ftp", ep2, "inspire_ftp", 6)
        keep2 = REPO / "logs/overnight/example_episode_inspire_ftp"
        if keep2.exists():
            shutil.rmtree(keep2)
        shutil.copytree(ep2, keep2)
        record("inspire_ftp: example episode kept in logs/overnight/", keep2.exists())
finally:
    fake_hand.terminate()
    try:
        fake_hand.wait(timeout=5)
    except subprocess.TimeoutExpired:
        fake_hand.kill()

# ============================================================== the two differ
print("\n=== (3) the two episode kinds are distinguishable on disk ===")
a = REPO / "logs/overnight/example_episode_arm_only/data.json"
b = REPO / "logs/overnight/example_episode_inspire_ftp/data.json"
if a.exists() and b.exists():
    da, db = json.loads(a.read_text()), json.loads(b.read_text())
    have_items = bool(da.get("data")) and bool(db.get("data"))
    record("both example episodes actually contain items", have_items,
           f"arm_only={len(da.get('data', []))} inspire={len(db.get('data', []))}")
    record("an arms-only episode says so, and a hand episode says so",
           da["info"]["source"]["end_effector"]["present"] is False
           and db["info"]["source"]["end_effector"]["present"] is True,
           "this is what upstream could not express")
    if have_items:
        record("arms-only ee arrays are empty, not zero-filled",
               da["data"][0]["states"]["left_ee"]["qpos"] == []
               and len(db["data"][0]["states"]["left_ee"]["qpos"]) == 6)
        record("no colors survive when there is no image server (and that is fine)",
               da["data"][0]["colors"] == {} and db["data"][0]["colors"] == {},
               "the joint data is what matters offline")
else:
    record("both example episodes exist to compare", False,
           f"arm_only={a.exists()} inspire={b.exists()}")

print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
for d in (tmp1,):
    shutil.rmtree(d, ignore_errors=True)
sys.exit(1 if n_fail else 0)
