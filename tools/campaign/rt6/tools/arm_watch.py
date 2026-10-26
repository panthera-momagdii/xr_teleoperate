#!/usr/bin/env python3
"""RT6: per-second arm log on DOMAIN 0, READERS ONLY (no DDS writer in this file).

Reads rt/lowstate (measured arm q, dq, motor temperature) and rt/lowcmd (the arm targets the
launcher publishes). Writes one CSV row per second: tracking error max|q_cmd - q_meas| (rad),
max |dq| (rad/s), hottest arm motor (C), and sample counts.
RT6b: shoulder pitch torque columns. RT6f: shoulder pitch/roll/yaw and elbow angles for both
arms (mean measured q over the second, rad), so the pose behind every reading is known.
Temperature rule (Mohammed, RT6b): WARN at >= 55 C; at >= 60 C for 2 consecutive seconds it
sends [q] to the launcher's tmux pane ONCE (`tmux send-keys -t rt6 q`) -- an orderly stop
(go home, exit), the only action this tool can take.
RT7 (23 Oct, Mohammed; Unitree: 90 C housing, 130 C winding, stay 10 C under; LowState
temperature[0] = housing, [1] = winding, SDK terminations.hpp): both values are watched on
every arm motor, each with its own thresholds. Housing [0]: WARN >= 75 C, STOP >= 80 C.
Winding [1]: WARN >= 110 C, STOP >= 120 C. STOP = 2 consecutive seconds at or over either
stop value, then [q] once, as before. These are hard limits: a stop value above 80 / 120
C, or the old single-value ARM_WATCH_WARN_C / ARM_WATCH_STOP_C, refuses to start. Until
RT6h this tool watched max(temperature[0], temperature[1]) against one threshold.
RT7 chatter (owner, 23 Oct, after the wrists chattered at full torque in the 20:16 lowering):
on every arm motor, per second, dq sign flips (|dq| > 0.02 rad/s) and the tau_est swing (max
- min). CHATTER = one motor with >= ARM_WATCH_CHATTER_FLIPS (100) flips AND a swing >=
ARM_WATCH_CHATTER_TAU_NM (5 Nm) in the same second (20:16: 300 to 500 flips, 25 to 52 Nm; at
rest: under 30 flips, 0.1 Nm). Then, once: the release file ARM_WATCH_RELEASE_FILE is written
(the launcher's controller polls it and goes to zero torque at once) and [q] is sent."""
import os, subprocess, sys, time
import numpy as np
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
out_path, pane = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "rt6")
DOMAIN = int(sys.argv[3]) if len(sys.argv) > 3 else 0          # 1 only for the fakes test
IFACE = sys.argv[4] if len(sys.argv) > 4 else "eth0"
# RT6b: 55 / 60 (was 60 / 75 in RT6). RT6e: one value per run, ARM_WATCH_WARN_C / _STOP_C.
# RT7: housing [0] and winding [1] separately; lower values allowed, higher never.
HARD = {"HOUSING": 80, "WINDING": 120}
if "ARM_WATCH_WARN_C" in os.environ or "ARM_WATCH_STOP_C" in os.environ:
    sys.exit("REFUSED: ARM_WATCH_WARN_C / ARM_WATCH_STOP_C are gone (RT7); use ARM_WATCH_HOUSING_WARN_C, "
             "ARM_WATCH_HOUSING_STOP_C, ARM_WATCH_WINDING_WARN_C, ARM_WATCH_WINDING_STOP_C")
H_WARN = int(os.environ.get("ARM_WATCH_HOUSING_WARN_C", "75"))
H_STOP = int(os.environ.get("ARM_WATCH_HOUSING_STOP_C", "80"))
W_WARN = int(os.environ.get("ARM_WATCH_WINDING_WARN_C", "110"))
W_STOP = int(os.environ.get("ARM_WATCH_WINDING_STOP_C", "120"))
if H_STOP > HARD["HOUSING"] or W_STOP > HARD["WINDING"] or H_WARN > H_STOP or W_WARN > W_STOP:
    sys.exit(f"REFUSED: thresholds housing {H_WARN}/{H_STOP} C, winding {W_WARN}/{W_STOP} C: stop values may not "
             f"exceed {HARD['HOUSING']} / {HARD['WINDING']} C and warn may not exceed stop")
CH_FLIPS = int(os.environ.get("ARM_WATCH_CHATTER_FLIPS", "100"))
CH_TAU = float(os.environ.get("ARM_WATCH_CHATTER_TAU_NM", "5"))
REL_FILE = os.environ.get("ARM_WATCH_RELEASE_FILE", "").strip() or None
print(f"thresholds: housing [0] warn {H_WARN} C, stop {H_STOP} C; winding [1] warn {W_WARN} C, stop {W_STOP} C; "
      f"q after 2 s at a stop value (pane {pane}); chatter: >= {CH_FLIPS} dq sign flips and >= {CH_TAU:g} Nm tau swing "
      f"in one second on one motor -> release file {'set' if REL_FILE else 'NOT SET (q only)'} and q", flush=True)
ARM = list(range(15, 29))
dom = Domain(DOMAIN, ChannelConfigHasInterface.replace("$__IF_NAME__$", IFACE)); dp = DomainParticipant(DOMAIN)
qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(256))
rs = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos=qos)
rc = DataReader(dp, Topic(dp, "rt/lowcmd", LowCmd_), qos=qos)
fh = open(out_path, "a", buffering=1)
fh.write("time,lowstate_n,lowcmd_n,track_err_max_rad,track_err_mean_rad,track_err_joint,dq_absmax_rad_s,dq_joint,temp_max_C,temp_joint,note,"
         "spL_tau_est_Nm,spL_tau_ff_Nm,spL_pd_est_Nm,spL_err_rad,spL_temp_C,spR_tau_est_Nm,spR_tau_ff_Nm,spR_pd_est_Nm,spR_err_rad,spR_temp_C,spL_kp,"
         "L_sh_pitch_rad,L_sh_roll_rad,L_sh_yaw_rad,L_elbow_rad,R_sh_pitch_rad,R_sh_roll_rad,R_sh_yaw_rad,R_elbow_rad,"
         "housing_max_C,housing_joint,winding_max_C,winding_joint,spL_housing_C,spL_winding_C,spR_housing_C,spR_winding_C,"
         + ",".join(f"dqmax_{i}" for i in range(15, 29)) + ","
         + ",".join(f"wrist_rev_{i}" for i in (19, 20, 21, 26, 27, 28)) + ","
         + ",".join(f"wrist_taupp_{i}" for i in (19, 20, 21, 26, 27, 28)) + ","
         "chat_flips_max,chat_flips_joint,chat_taupp_max,chat_taupp_joint,chatter\n")
last_cmd = None; hot = 0; q_sent = False; warned = {"housing": False, "winding": False}
WRIST = (19, 20, 21, 26, 27, 28)   # RT7: wrist roll, pitch, yaw; left then right
last_dq_sign = {i: 0 for i in WRIST}; rev = {i: 0 for i in WRIST}
tau_lo = {i: float("inf") for i in WRIST}; tau_hi = {i: float("-inf") for i in WRIST}   # RT7: tau_est swing per second
c_sign = {i: 0 for i in ARM}; c_rev = {i: 0 for i in ARM}                                 # RT7 chatter: all 14 arm motors
c_lo = {i: float("inf") for i in ARM}; c_hi = {i: float("-inf") for i in ARM}
chatter_done = False
SP = (15, 22)                      # shoulder pitch L, R (RT6b torque log)
sp_cmd = None                      # {id: (q, tau_ff, kp, kd)} from the last lowcmd
sp_acc = {i: [] for i in SP}       # per second: (tau_est, tau_ff, pd_est, err, temp)
sp_kp = float('nan')
ANG = (15, 16, 17, 18, 22, 23, 24, 25)   # RT6f: shoulder pitch, roll, yaw, elbow; left then right
ang_acc = []
nxt = time.monotonic() + 1.0
errs, dqs, temps, ns, nc = [], [], [], 0, 0
t0s, t1s = [], []                  # RT7: per sample, temperature[0] and [1] of the 14 arm motors
sp01 = {i: [] for i in SP}         # RT7: per sample, (temperature[0], temperature[1]) at shoulder pitch
while True:
    for s in rc.take(N=256):
        if not hasattr(s, "motor_cmd"):      # InvalidSample: a writer went away (RT6 crash)
            continue
        last_cmd = np.array([s.motor_cmd[i].q for i in ARM]); nc += 1
        sp_cmd = {i: (s.motor_cmd[i].q, s.motor_cmd[i].tau, s.motor_cmd[i].kp, s.motor_cmd[i].kd) for i in SP}
    for s in rs.take(N=256):
        if not hasattr(s, "motor_state"):
            continue
        ns += 1
        q = np.array([s.motor_state[i].q for i in ARM]); dq = np.array([s.motor_state[i].dq for i in ARM])
        t = np.array([max(s.motor_state[i].temperature) for i in ARM])
        dqs.append(np.abs(dq)); temps.append(t)
        t0s.append([s.motor_state[i].temperature[0] for i in ARM]); t1s.append([s.motor_state[i].temperature[1] for i in ARM])
        for i in SP:
            sp01[i].append((s.motor_state[i].temperature[0], s.motor_state[i].temperature[1]))
        for i in ARM:                  # RT7 chatter detector inputs
            mm_ = s.motor_state[i]
            c_lo[i] = min(c_lo[i], mm_.tau_est); c_hi[i] = max(c_hi[i], mm_.tau_est)
            sg = 1 if mm_.dq > 0.02 else (-1 if mm_.dq < -0.02 else 0)
            if sg != 0:
                if c_sign[i] != 0 and sg != c_sign[i]:
                    c_rev[i] += 1
                c_sign[i] = sg
        for i in WRIST:                # buzz indicator: dq sign reversals while moving, tau_est swing
            te = s.motor_state[i].tau_est
            tau_lo[i] = min(tau_lo[i], te); tau_hi[i] = max(tau_hi[i], te)
            v = s.motor_state[i].dq
            sg = 1 if v > 0.02 else (-1 if v < -0.02 else 0)
            if sg != 0:
                if last_dq_sign[i] != 0 and sg != last_dq_sign[i]:
                    rev[i] += 1
                last_dq_sign[i] = sg
        ang_acc.append([s.motor_state[i].q for i in ANG])
        if last_cmd is not None:
            errs.append(np.abs(last_cmd - q))
        if sp_cmd is not None:
            for i in SP:
                qc, tff, kp, kd = sp_cmd[i]; m = s.motor_state[i]
                err = qc - m.q
                sp_acc[i].append((m.tau_est, tff, kp * err - kd * m.dq, err, max(m.temperature)))
            sp_kp = sp_cmd[15][2]
    if time.monotonic() >= nxt:
        nxt += 1.0
        note = ""
        e = np.array(errs) if errs else None; d = np.array(dqs) if dqs else None; T = np.array(temps) if temps else None
        em = float(e.max()) if e is not None else float("nan"); ea = float(e.mean()) if e is not None else float("nan")
        ej = int(ARM[int(e.max(0).argmax())]) if e is not None else -1
        dm = float(d.max()) if d is not None else float("nan"); dj = int(ARM[int(d.max(0).argmax())]) if d is not None else -1
        tm = int(T.max()) if T is not None else -1; tj = int(ARM[int(T.max(0).argmax())]) if T is not None else -1
        A0 = np.array(t0s) if t0s else None; A1 = np.array(t1s) if t1s else None
        h = int(A0.max()) if A0 is not None else -1; hj = int(ARM[int(A0.max(0).argmax())]) if A0 is not None else -1
        w = int(A1.max()) if A1 is not None else -1; wj = int(ARM[int(A1.max(0).argmax())]) if A1 is not None else -1
        over = []
        if h >= H_STOP:
            over.append(f"housing [0] {h} C motor {hj}")
        if w >= W_STOP:
            over.append(f"winding [1] {w} C motor {wj}")
        hot = hot + 1 if over else 0
        notes = []
        if h >= H_WARN and not warned["housing"]:
            warned["housing"] = True; notes.append(f"WARN temp housing [0] {h} C motor {hj}")
        if w >= W_WARN and not warned["winding"]:
            warned["winding"] = True; notes.append(f"WARN temp winding [1] {w} C motor {wj}")
        if hot >= 2 and not q_sent:
            q_sent = True
            subprocess.run(["tmux", "send-keys", "-t", pane, "q"], stdin=subprocess.DEVNULL, capture_output=True)
            notes.append(f"STOP temp {' and '.join(over)} for 2 s: sent q to tmux {pane}")
        sw = {i: (c_hi[i] - c_lo[i]) if c_hi[i] >= c_lo[i] else 0.0 for i in ARM}
        fj = max(ARM, key=lambda i: c_rev[i]); tj_ = max(ARM, key=lambda i: sw[i])
        chat = [i for i in ARM if c_rev[i] >= CH_FLIPS and sw[i] >= CH_TAU]
        chat_txt = " ".join(f"m{i}:{c_rev[i]}/{sw[i]:.1f}" for i in chat)
        if chat and not chatter_done:
            chatter_done = True
            why = f"CHATTER motor {chat[0]}: {c_rev[chat[0]]} dq sign flips, {sw[chat[0]]:.1f} Nm tau swing in 1 s"
            if REL_FILE:
                try:
                    with open(REL_FILE, "w") as _f:
                        _f.write(why + "\n")
                except OSError as _e:
                    why += f" (release file NOT written: {_e})"
            if not q_sent:
                q_sent = True
                subprocess.run(["tmux", "send-keys", "-t", pane, "q"], stdin=subprocess.DEVNULL, capture_output=True)
            notes.append(why + (": release file written" if REL_FILE else "") + f", q sent to tmux {pane}")
        chat_cols = f"{c_rev[fj]},{fj},{sw[tj_]:.2f},{tj_},{chat_txt}"
        c_rev = {i: 0 for i in ARM}; c_lo = {i: float("inf") for i in ARM}; c_hi = {i: float("-inf") for i in ARM}
        note = "; ".join(notes)
        spc = []
        for i in SP:
            a = np.array(sp_acc[i]) if sp_acc[i] else None
            spc += ([f"{a[:, 0].mean():.2f}", f"{a[:, 1].mean():.2f}", f"{a[:, 2].mean():.2f}", f"{a[:, 3].mean():.4f}", f"{int(a[:, 4].max())}"]
                    if a is not None else ["nan"] * 5)
            sp_acc[i] = []
        ang = (",".join(f"{v:.4f}" for v in np.mean(np.array(ang_acc), axis=0)) if ang_acc else ",".join(["nan"] * len(ANG)))
        ang_acc = []
        spt = []
        for i in SP:
            a = np.array(sp01[i]) if sp01[i] else None
            spt += [f"{int(a[:, 0].max())}", f"{int(a[:, 1].max())}"] if a is not None else ["nan", "nan"]
            sp01[i] = []
        dqj = ",".join(f"{v:.3f}" for v in d.max(0)) if d is not None else ",".join(["nan"] * 14)
        revs = ",".join(str(rev[i]) for i in WRIST)
        rev = {i: 0 for i in WRIST}
        tpp = ",".join((f"{tau_hi[i] - tau_lo[i]:.2f}" if tau_hi[i] >= tau_lo[i] else "nan") for i in WRIST)
        tau_lo = {i: float("inf") for i in WRIST}; tau_hi = {i: float("-inf") for i in WRIST}
        fh.write(f"{time.strftime('%T')},{ns},{nc},{em:.4f},{ea:.4f},{ej},{dm:.3f},{dj},{tm},{tj},{note.replace(',', ' ')},{','.join(spc)},"
                 f"{sp_kp:.0f},{ang},{h},{hj},{w},{wj},{','.join(spt)},{dqj},{revs},{tpp},{chat_cols}\n")
        if note:
            print(note, flush=True)
        errs, dqs, temps, ns, nc = [], [], [], 0, 0
        t0s, t1s = [], []
    time.sleep(0.005)
