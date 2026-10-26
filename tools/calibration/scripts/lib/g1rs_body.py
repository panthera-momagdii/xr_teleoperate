#!/usr/bin/env python3
"""g1rs_20261012 passive body monitor (DataReaders only, no publish): rt/lowcmd, rt/lowstate, rt/lf/bmsstate (unitree_hg types).
Stop conditions it reports (Mohammed 2026-10-12, 3-sample filter): a second rt/lowcmd writer, or any nonzero kp, kd or tau on the
29 motors. Arm or wrist motion (joints 15 to 28 more than 0.02 rad from the baseline) and a stale lowstate are events only.
Interface as h1_single.BodyMonitor: start(), set_baseline(), battery(), check(), pop_events(), close()."""
import os
import sys
import threading
import time

ARM_JOINTS = range(15, 29)
ARM_TOL_RAD, STALE_S = 0.02, 0.5


class BodyMonitor(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.qos import Policy, Qos
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        sys.path.insert(0, os.path.expanduser("~/unitree_sdk2_python"))
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowCmd_, LowState_
        dp = DomainParticipant(0)
        q = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8))
        self.rc = DataReader(dp, Topic(dp, "rt/lowcmd", LowCmd_), qos=q)
        self.rs = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos=q)
        self.rb = DataReader(dp, Topic(dp, "rt/lf/bmsstate", BmsState_), qos=q)
        self.dp, self.lock, self.stop_evt = dp, threading.Lock(), threading.Event()
        self.q = self.t_q = self.soc = self.base = self.trip = None
        self.writers, self.n_gain, self.n_second = {}, 0, 0
        self.events, self.arm_out, self.stale = [], False, False

    def run(self):
        while not self.stop_evt.is_set():
            for s in self.rc.take(N=16):
                h = s.sample_info.publication_handle
                with self.lock:
                    self.writers[h] = self.writers.get(h, 0) + 1
                    m = s.motor_cmd[:29]
                    bad = any(x.kp != 0 or x.kd != 0 or x.tau != 0 for x in m)
                    self.n_gain = self.n_gain + 1 if bad else 0
                    if self.n_gain >= 3 and self.trip is None:
                        self.trip = ("LOWCMD_NONZERO_KP_KD_TAU", {"kp_max": max(abs(x.kp) for x in m), "kd_max": max(abs(x.kd) for x in m),
                                                                  "tau_absmax": max(abs(x.tau) for x in m)})
                    if len([k for k, n in self.writers.items() if n >= 3]) > 1 and self.trip is None:
                        self.trip = ("LOWCMD_SECOND_WRITER", {"writers": len(self.writers)})
            for s in self.rs.take(N=1):
                q, now = [m.q for m in s.motor_state[:29]], time.monotonic()
                with self.lock:
                    self.q, self.t_q = q, now
                    if self.base is not None:
                        dq = max(abs(q[j] - self.base[j]) for j in ARM_JOINTS)
                        if dq > ARM_TOL_RAD and not self.arm_out:
                            self.arm_out = True
                            self.events.append(("ARM_MOVED", {"dq_rad_max": round(dq, 4)}))
                        elif dq <= ARM_TOL_RAD and self.arm_out:
                            self.arm_out = False
                            self.events.append(("ARM_BACK", {"dq_rad_max": round(dq, 4)}))
            for s in self.rb.take(N=1):
                with self.lock:
                    self.soc = int(s.soc)
            with self.lock:
                st = self.t_q is not None and time.monotonic() - self.t_q > STALE_S
                if st != self.stale:
                    self.stale = st
                    self.events.append(("LOWSTATE_STALE" if st else "LOWSTATE_BACK", {}))
            time.sleep(0.002)

    def set_baseline(self):
        rows, t_end = [], time.monotonic() + 0.3
        while time.monotonic() < t_end:
            with self.lock:
                if self.q is not None:
                    rows.append(self.q)
            time.sleep(0.01)
        with self.lock:
            self.base = [sorted(c)[len(c) // 2] for c in zip(*rows)] if rows else None
            self.arm_out = False
        if not rows:
            self.events.append(("LOWSTATE_NOT_READY_AT_BASELINE", {}))

    def battery(self):
        with self.lock:
            return self.soc

    def check(self):
        with self.lock:
            return self.trip

    def pop_events(self):
        with self.lock:
            e, self.events = self.events, []
        return e

    def summary(self):
        with self.lock:
            return {"lowcmd_writers": len(self.writers), "soc": self.soc, "trip": self.trip}

    def close(self):
        self.stop_evt.set()
        self.join(timeout=1.0)
