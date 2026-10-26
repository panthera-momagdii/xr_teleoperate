#!/usr/bin/env python3
"""g1rs_20261012 register burst rule (Mohammed 2026-10-12, both hands, every tool):
* A burst on an axis = consecutive reads where that axis shows a nonzero ERROR byte or a value that is not physical (TEMP byte 0 or
  above 100 C, |CURRENT| >= 3000 mA). Ordinary TEMP jumps inside 1 to 100 C stay TEMP_SPIKE events (2 s median rule).
* ERROR stops only when nonzero on 6 consecutive reads (two register refreshes).
* A shorter burst is a GLITCH_BURST event with all registers of that axis for every read of the burst; bursts are counted per axis.
* Stop if one axis logs more than 10 bursts in a run, or if a burst comes with any change in ANGLE_ACT or POS_ACT on that axis
  (against the read just before the burst).
update(rec) returns None or (stop_reason, detail)."""
import os

GROUPS = ("ERROR", "STATUS", "TEMP", "CURRENT", "ANGLE_ACT", "POS_ACT", "FORCE_ACT")


class BurstTracker:
    def __init__(self, ev, hand):
        self.ev, self.hand = ev, hand            # ev(name, **kw) writes an event line
        self.cur = [None] * 6
        self.count = [0] * 6
        self.err_run = [0] * 6
        self.prev = [None] * 6

    @staticmethod
    def axis_regs(rec, k):
        d = rec["dec"]
        return {"seq": rec["seq"], "t_mono_ns": rec["t_mono_ns"], "error": d["error"][k], "status": d["status"][k], "temp": d["temp"][k],
                "current": d["current"][k], "angle_act": d["angle_act"][k], "pos_act": d["pos_act"][k], "force_act": d["force_act"][k],
                "raw": {g: rec["raw"][g] for g in GROUPS}}

    def update(self, rec):
        d = rec["dec"]
        stop = None
        for k in range(6):
            bad = d["error"][k] != 0 or d["temp"][k] == 0 or d["temp"][k] > 100 or abs(d["current"][k]) >= 3000
            self.err_run[k] = self.err_run[k] + 1 if d["error"][k] != 0 else 0
            if self.err_run[k] >= 6 and stop is None:
                stop = ("ERROR_NONZERO_6_READS", {"hand": self.hand, "axis": k, "error": d["error"]})
            b = self.cur[k]
            if bad:
                if b is None:
                    b = self.cur[k] = {"axis": k, "before": self.prev[k], "reads": []}
                b["reads"].append(self.axis_regs(rec, k))
            elif b is not None:
                self.cur[k] = None
                self.count[k] += 1
                ref = b["before"]
                moved = ref is not None and any(r["angle_act"] != ref[0] or r["pos_act"] != ref[1] for r in b["reads"] + [self.axis_regs(rec, k)])
                self.ev("GLITCH_BURST", axis=k, n_reads=len(b["reads"]), burst_no=self.count[k], before_angle_pos=ref, moved=moved, reads=b["reads"],
                        after=self.axis_regs(rec, k))
                if moved and stop is None and os.environ.get("CW_BURST_MOVE_LOG_ONLY") != "1":   # 16 Oct: logged only (Mohammed)
                    stop = ("BURST_WITH_ANGLE_OR_POS_CHANGE", {"hand": self.hand, "axis": k, "before": ref,
                                                               "burst": [(r["angle_act"], r["pos_act"]) for r in b["reads"]]})
                if self.count[k] > 10 and stop is None:
                    stop = ("MORE_THAN_10_BURSTS_ON_AXIS", {"hand": self.hand, "axis": k, "count": self.count[k]})
            if not bad:
                self.prev[k] = (d["angle_act"][k], d["pos_act"][k])
        return stop
