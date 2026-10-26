#!/usr/bin/env python3
"""RT1 step 8: per-solve timing of G1_29_ArmIK on ik_smoke's own seeded targets.

ik_smoke prints mean and max only. This calls the same solver (G1_29_ArmIK,
Unit_Test=True, warm-started solve_ik) on the same targets (tools.ik_smoke.oval_targets,
same steps and seed) and reports the first solve separately plus p50 / p99 / max.
No DDS, no network. Run with cwd = ~/xrt/teleop/robot_control and no model cache.
"""
import argparse, json, os, sys, time
import numpy as np
XRT = os.path.expanduser("~/xrt")
sys.path.append(XRT)
from tools.ik_smoke import oval_targets  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--steps", type=int, default=2000)
ap.add_argument("--seed", type=int, default=20260827)
ap.add_argument("--label", default="run")
ap.add_argument("--out", required=True)
a = ap.parse_args()
if os.path.exists("g1_29_model_cache.pkl"):
    print("model cache exists; delete it first so both tools build the model the same way")
    sys.exit(2)
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK  # noqa: E402
t0 = time.perf_counter()
ik = G1_29_ArmIK(Unit_Test=True, Visualization=False)
build_s = time.perf_counter() - t0
targets = oval_targets(a.steps, a.seed)
ms = []
for left, right in targets:
    t = time.perf_counter()
    ik.solve_ik(left, right)
    ms.append((time.perf_counter() - t) * 1000.0)
ms = np.array(ms)
rest = ms[1:]
def stats(x):
    return {"n": int(len(x)), "mean": float(x.mean()), "p50": float(np.percentile(x, 50)),
            "p90": float(np.percentile(x, 90)), "p99": float(np.percentile(x, 99)),
            "max": float(x.max()), "min": float(x.min()),
            "over_10ms": int((x > 10).sum()), "over_33ms": int((x > 33.3).sum())}
res = {"label": a.label, "steps": a.steps, "seed": a.seed, "model_build_s": build_s,
       "first_solve_ms": float(ms[0]), "rest": stats(rest), "all": stats(ms),
       "loadavg_end": os.getloadavg(), "captured_at": time.strftime("%Y-%m-%d %H:%M:%S %Z")}
print(f"[{a.label}] model build {build_s:.2f} s; first solve {ms[0]:.2f} ms")
for k in ("rest", "all"):
    s = res[k]
    print(f"[{a.label}] {k:4s} n={s['n']} mean {s['mean']:.3f}  p50 {s['p50']:.3f}  p90 {s['p90']:.3f}  "
          f"p99 {s['p99']:.3f}  max {s['max']:.3f} ms  >10ms {s['over_10ms']}  >33.3ms {s['over_33ms']}")
with open(a.out, "w") as fh:
    json.dump(res, fh, indent=2)
np.save(a.out.replace(".json", "_ms.npy"), ms)
sys.stdout.flush()
os._exit(0)
