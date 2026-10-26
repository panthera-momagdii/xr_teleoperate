#!/usr/bin/env python3
"""RT6d: shoulder pitch heating analysis (G1, RT6 / RT6b / RT6c). CPU only.

Reads (never writes) the watcher logs under rt6/out/ and writes tables, JSON and figures to
rt6d/out/heat/ and the page figures to reports/RT6_heat_analysis_20261022/.
No DDS, no robot, no network.

Run: PYTHONNOUSERSITE=1 ~/miniconda3/envs/xrt/bin/python -B rt6d/tools/heat_analysis.py

Conventions
- Clock: PC2 local time (CST). Times are seconds from 22 Oct 00:00:00; 21 Oct times are negative.
- Watcher row k: one window of about 1 s, mean torque and MAX integer temperature over the window.
  The watcher paces rows on a monotonic 1 s grid (nxt += 1.0), so rows are exactly 1 s apart
  plus a small processing lag. Absolute row time is calibrated on the STOP row: the watcher
  sent [q] to tmux and wrote that row at once, and the launcher logged "[key] q received"
  at a known time. Row window = (t_k - 1 s, t_k].
- Model reading for row k = max of the model temperature over the row window (same as the logger).
"""
import csv, json, math, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, brentq
from scipy.linalg import expm
from scipy.signal import lfilter
from scipy import stats

CAMP = "/home/unitree/panthera/remote-teleop-20261012"
IN = CAMP + "/rt6/out"
OUT = CAMP + "/rt6d/out/heat"
FIG = CAMP + "/reports/RT6_heat_analysis_20261022"
os.makedirs(OUT, exist_ok=True); os.makedirs(FIG, exist_ok=True)

# palette (dataviz reference palette, light mode)
C_L, C_R, C_3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
                     "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
                     "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
                     "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
                     "legend.frameon": False, "lines.linewidth": 2})


def sec(s):
    """'hh:mm:ss[.ff]' PC2 local -> seconds from 22 Oct 00:00 (21 Oct evening is negative)."""
    h, m, x = s.split(":")
    v = int(h) * 3600 + int(m) * 60 + float(x)
    return v if int(h) < 12 else v - 86400


def hms(t):
    t = t % 86400
    return "%02d:%02d:%05.2f" % (t // 3600, (t % 3600) // 60, t % 60)


# ----------------------------------------------------------------------------- events
# from rt6/out/session*/launcher_pane.log and reports RT6 / RT6b / RT6c
EV = {
    "RT6": dict(file="session/arm_watch.csv", r=sec("23:44:26.749"), follow=sec("23:44:32.0"),
                q=sec("23:45:02.279"), release=sec("23:45:08.147"), exit=sec("23:45:08.147")),
    "RT6b": dict(file="session2/arm_watch.csv", r=sec("00:25:42.84"), follow=sec("00:25:47.85"),
                 q=sec("00:26:08.04"), lower_end=sec("00:26:10.75"), release=sec("00:26:12.80"),
                 exit=sec("00:26:13.64"), kp_on=sec("00:22:31.44")),
    "RT6c": dict(file="session3/arm_watch.csv", r=sec("00:38:19.69"), r_first=sec("00:38:13.74"),
                 follow=sec("00:38:21.71"), q=sec("00:38:40.60"), lower_end=sec("00:38:42.91"),
                 release=sec("00:38:44.91"), exit=sec("00:38:45.77"), kp_on=sec("00:34:19.42")),
}
# spot readings that exist only in the reports (temp_read.py printed to the terminal, not saved)
SPOT = {("RT6b", "L"): (sec("00:28:40"), 51, "reports/RT6b_20261022.md section 4"),
        ("RT6c", "L"): (sec("00:39:26"), 52, "reports/RT6c_20261022.md section 5"),
        ("RT6c", "R"): (sec("00:39:26"), 49, "reports/RT6c_20261022.md section 5")}


def f2(x):
    try:
        return float(x)
    except ValueError:
        return float("nan")


def load_watch(name):
    ev = EV[name]
    rows = list(csv.DictReader(open(os.path.join(IN, ev["file"]))))
    k_stop = [k for k, r in enumerate(rows) if r["note"].startswith("STOP")][0]
    k = np.arange(len(rows))
    t = ev["q"] + 0.01 + (k - k_stop)            # STOP row written about 10 ms after the tmux send
    lab = np.array([sec(r["time"]) for r in rows])
    d = dict(name=name, t=t, label=lab, k_stop=k_stop, rows=rows,
             temp_max=np.array([f2(r["temp_max_C"]) for r in rows]),
             temp_joint=np.array([int(r["temp_joint"]) for r in rows]))
    if "spL_tau_est_Nm" in rows[0]:
        for s in "LR":
            d["tau" + s] = np.array([f2(r[f"sp{s}_tau_est_Nm"]) for r in rows])
            d["ff" + s] = np.array([f2(r[f"sp{s}_tau_ff_Nm"]) for r in rows])
            d["pd" + s] = np.array([f2(r[f"sp{s}_pd_est_Nm"]) for r in rows])
            d["T" + s] = np.array([f2(r[f"sp{s}_temp_C"]) for r in rows])
        d["kp"] = np.array([f2(r["spL_kp"]) for r in rows])
    # sanity: the calibrated time must floor to the label within 1 s
    off = lab - np.floor(t)
    d["label_check"] = dict(min=float(off.min()), max=float(off.max()))
    return d


W = {n: load_watch(n) for n in EV}


def phase_of(name, tm):
    """phase at time tm (window midpoint)."""
    e = EV[name]
    if tm < e["r"]:
        return "hold"
    if tm < e["follow"]:
        return "start/blend"
    if tm < e["q"]:
        return "follow"
    if name == "RT6":
        return "go-home" if tm < e["exit"] else "released"
    if tm < e["release"]:
        return "lower/release"
    return "released"


# ----------------------------------------------------------------------------- baselines
def baseline():
    out = {}
    for n in ("RT6b", "RT6c"):
        d = W[n]; e = EV[n]
        m = (d["t"] > e["r"] - 60) & (d["t"] <= e["r"])
        for s in "LR":
            out[(n, s)] = float(np.median(d["T" + s][m]))
    return out


T0 = baseline()

cool_log = []
for line in open(os.path.join(IN, "cooldown_log.txt")):
    mm = re.match(r"(\d\d:\d\d:\d\d) shoulder_pitch L (\d+) R (\d+) C \| arm max (\d+) C \(motor (\d+)\)", line)
    if mm:
        cool_log.append(dict(time=mm.group(1), t=sec(mm.group(1)), L=int(mm.group(2)), R=int(mm.group(3)),
                             arm_max=int(mm.group(4)), arm_max_motor=int(mm.group(5))))
census = {}
for f in ("census_start.json", "census_end.json", "census_after_reboot.json"):
    j = json.load(open(os.path.join(IN, f)))
    ca = j["captured_at"]; tt = sec(ca[9:11] + ":" + ca[11:13] + ":" + ca[13:15])
    census[f] = dict(t_end=tt, window_s=j["window_s"], L=j["arm_motor_temperature_max_c"]["15"],
                     R=j["arm_motor_temperature_max_c"]["22"], time=hms(tt)[:8])
br = list(csv.DictReader(open(os.path.join(IN, "session2/arm_watch_before_restart.csv"))))
br_max = [int(r["temp_max_C"]) for r in br]; br_joint = sorted(set(int(r["temp_joint"]) for r in br))

rest = []   # resting / low-torque baseline table
d = W["RT6b"]; m = d["kp"] == 0; m &= d["t"] < EV["RT6b"]["kp_on"]
rest.append(dict(what="RT6b watcher, damping mode before launch", window=f"{hms(d['t'][m][0])[:8]} to {hms(d['t'][m][-1])[:8]}",
                 L=f"{int(d['TL'][m].min())} to {int(d['TL'][m].max())}", R=f"{int(d['TR'][m].min())} to {int(d['TR'][m].max())}",
                 tau=f"{np.nanmean(np.abs(d['tauL'][m])):.2f} / {np.nanmean(np.abs(d['tauR'][m])):.2f}", src="rt6/out/session2/arm_watch.csv"))
for n in ("RT6b", "RT6c"):
    d = W[n]; e = EV[n]; m = (d["kp"] == 80) & (d["t"] < e.get("r_first", e["r"]))
    rest.append(dict(what=f"{n} hold before [r], kp 80", window=f"{hms(d['t'][m][0])[:8]} to {hms(d['t'][m][-1])[:8]}",
                     L=f"{int(d['TL'][m].min())} to {int(d['TL'][m].max())}", R=f"{int(d['TR'][m].min())} to {int(d['TR'][m].max())}",
                     tau=f"{np.mean(np.abs(d['tauL'][m])):.2f} / {np.mean(np.abs(d['tauR'][m])):.2f}", src=f"rt6/out/{e['file']}"))

# ----------------------------------------------------------------------------- 1. time series
ts_rows = []
for n in ("RT6", "RT6b", "RT6c"):
    d = W[n]; e = EV[n]
    for k, tk in enumerate(d["t"]):
        if tk < e["r"] - 30 or tk > e["release"] + 60:
            continue
        row = dict(run=n, row_label=d["rows"][k]["time"], t_end=round(tk, 2), t_from_r_s=round(tk - e["r"], 2),
                   t_from_release_s=round(tk - e["release"], 2), phase=phase_of(n, tk - 0.5),
                   temp_max_C=int(d["temp_max"][k]), temp_joint=int(d["temp_joint"][k]))
        if n != "RT6":
            for s in "LR":
                row["tau" + s + "_Nm"] = d["tau" + s][k]; row["ff" + s + "_Nm"] = d["ff" + s][k]
                row["pd" + s + "_Nm"] = d["pd" + s][k]; row["T" + s + "_C"] = d["T" + s][k]
            row["kp"] = d["kp"][k]
        ts_rows.append(row)


def write_csv(path, rows):
    keys = []
    for r in rows:
        for kk in r:
            if kk not in keys:
                keys.append(kk)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys); w.writeheader()
        for r in rows:
            w.writerow({kk: (("%.4g" % v) if isinstance(v, float) else v) for kk, v in r.items()})


write_csv(OUT + "/timeseries.csv", ts_rows)


def shade(ax, n, rel=True):
    e = EV[n]; z = e["r"]
    spans = [(e["r"], e["follow"], "#d9d8d3", "start / blend"), (e["follow"], e["q"], "#f4d9cb", "FOLLOW"),
             (e["q"], e["release"], "#d6e6f7", "lower / release" if n != "RT6" else "go-home")]
    for a, b, c, lab in spans:
        ax.axvspan(a - z, b - z, color=c, alpha=0.7, lw=0, zorder=0)


def savefig(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=130, bbox_inches="tight")
    fig.savefig(os.path.join(FIG, name), dpi=130, bbox_inches="tight")
    plt.close(fig)


fig, axs = plt.subplots(2, 3, figsize=(12.5, 6.2), sharex="col", gridspec_kw=dict(height_ratios=[1.25, 1]))
for j, n in enumerate(("RT6", "RT6b", "RT6c")):
    d = W[n]; e = EV[n]; z = e["r"]
    m = (d["t"] >= e["r"] - 10) & (d["t"] <= e["release"] + 60)
    x = d["t"][m] - 0.5 - z
    ax = axs[0, j]; shade(ax, n)
    if n == "RT6":
        jt = d["temp_joint"][m]; T = d["temp_max"][m]
        for s, col, jid in (("L", C_L, 15), ("R", C_R, 22)):
            mk = jt == jid
            ax.plot(x[mk], T[mk], "o", ms=4.5, color=col, label=f"{'left' if s == 'L' else 'right'} shoulder pitch (when hottest)")
        mk = ~np.isin(jt, [15, 22])
        ax.plot(x[mk], T[mk], "o", ms=4, mfc="none", color=INK2, label="hottest arm motor is another joint")
        c = census["census_end.json"]
        ax.plot([c["t_end"] - c["window_s"] - z, c["t_end"] - z], [c["L"], c["L"]], color=C_L, lw=3, solid_capstyle="butt")
        ax.annotate(f"census {c['L']} C (max over {c['window_s']:.1f} s)", (c["t_end"] - z, c["L"]), xytext=(-150, 14),
                    textcoords="offset points", fontsize=8, color=INK2)
        ax.set_title("RT6, 21 Oct: hottest arm motor only", loc="left", fontsize=10)
    else:
        for s, col in (("L", C_L), ("R", C_R)):
            ax.step(d["t"][m] - z, d["T" + s][m], where="pre", color=col, lw=1.8,
                    label=f"{'left (15)' if s == 'L' else 'right (22)'} shoulder pitch")
        for (rn, s), (tt, v, src) in SPOT.items():
            if rn == n and tt - z < e["release"] - z + 60:
                ax.plot([tt - z], [v], "D", ms=6, mfc="none", color=C_L if s == "L" else C_R)
                ax.annotate("spot reading (report)", (tt - z, v), xytext=(-40, 10), textcoords="offset points", fontsize=7.5, color=INK2)
        ax.set_title(f"{n}, 22 Oct", loc="left", fontsize=10)
    ax.axhline(60, color=INK2, lw=1, ls="--"); ax.text(ax.get_xlim()[0] if False else -9, 60.4, "60 C project stop", fontsize=7.5, color=INK2)
    ax.axvline(e["release"] - z, color=INK2, lw=0.8, ls=":")
    ax.set_ylabel("temperature reading (C)") if j == 0 else None
    ax.legend(loc="upper left", fontsize=7.5)
    ax2 = axs[1, j]; shade(ax2, n)
    if n == "RT6":
        ax2.text(0.5, 0.5, "torque not logged in RT6", transform=ax2.transAxes, ha="center", color=INK2)
        ax2.set_ylim(-9, 1)
    else:
        for s, col in (("L", C_L), ("R", C_R)):
            ax2.step(d["t"][m] - z, d["tau" + s][m], where="pre", color=col, lw=1.8, label=f"{'left' if s == 'L' else 'right'} tau_est")
        ax2.legend(loc="lower left", fontsize=7.5)
    ax2.axvline(e["release"] - z, color=INK2, lw=0.8, ls=":")
    ax2.set_xlabel("seconds from [r]  (shaded: start/blend, FOLLOW, lowering; dotted: release)")
    ax2.set_ylabel("measured torque (Nm, window mean)") if j == 0 else None
    ax2.set_xlim(-10, e["release"] - z + 60)
fig.tight_layout()
savefig(fig, "fig1_timeseries.png")

# ----------------------------------------------------------------------------- models
DT = 0.05
NSUB = int(round(1.0 / DT))


def make_ds(n, s, t_from=None, t_to=None, extend_to=None):
    d = W[n]; e = EV[n]
    lo = e["r"] - 60 if t_from is None else t_from
    hi = d["t"][-1] if t_to is None else t_to
    m = (d["t"] >= lo) & (d["t"] <= hi)
    t = d["t"][m]; tau = d["tau" + s][m].copy(); T = d["T" + s][m].copy()
    tau[np.isnan(tau)] = 0.1
    if extend_to is not None and extend_to > t[-1]:
        ext = np.arange(t[-1] + 1, extend_to + 0.5, 1.0)
        t = np.concatenate([t, ext]); tau = np.concatenate([tau, np.full(len(ext), 0.12)])
        T = np.concatenate([T, np.full(len(ext), np.nan)])
    return dict(run=n, side=s, t=t, tau=tau, T=T, T0=T0[(n, s)])


def system(model, p):
    """continuous-time linear system: x' = A x + B g(u), reading = C x (relative to T0)."""
    if model == "M1":          # one node, a*tau^2
        a, tt = p; A = [[-1 / tt]]; B = [1.0]; C = [1.0]; g = lambda u: a * u ** 2
    elif model == "M2":        # one node, a*tau^2 + b*|tau|
        a, b, tt = p; A = [[-1 / tt]]; B = [1.0]; C = [1.0]; g = lambda u: a * u ** 2 + b * np.abs(u)
    elif model == "M2b":       # one node, b*|tau| only (for comparison)
        b, tt = p; A = [[-1 / tt]]; B = [1.0]; C = [1.0]; g = lambda u: b * np.abs(u)
    elif model == "M3":        # winding + housing, reading on the winding
        a, tw, r, th = p
        A = [[-1 / tw, 1 / tw], [1 / (r * tw), -1 / (r * tw) - 1 / th]]; B = [1.0, 0.0]; C = [1.0, 0.0]; g = lambda u: a * u ** 2
    elif model == "M4":        # one node + first-order reading delay
        a, tt, ts = p
        A = [[-1 / tt, 0.0], [1 / ts, -1 / ts]]; B = [1.0, 0.0]; C = [0.0, 1.0]; g = lambda u: a * u ** 2
    return np.array(A, float), np.array(B, float), np.array(C, float), g


def discretize(A, B, dt):
    nx = A.shape[0]
    M = np.zeros((nx + 1, nx + 1)); M[:nx, :nx] = A * dt; M[:nx, nx] = B * dt
    E = expm(M)
    return E[:nx, :nx], E[:nx, nx]


def run_lin(Ad, Bd, C, g, x0=None):
    """x_{i+1} = Ad x_i + Bd g_i ; returns y_i = C x_i for i = 0..len(g)."""
    nx = Ad.shape[0]
    x = np.zeros(nx) if x0 is None else np.array(x0, float)
    if nx == 1 and x0 is None:
        y = lfilter([0.0, Bd[0]], [1.0, -Ad[0, 0]], np.concatenate([g, [0.0]]))
        return C[0] * y
    y = np.empty(len(g) + 1); y[0] = C @ x
    for i, gi in enumerate(g):
        x = Ad @ x + Bd * gi; y[i + 1] = C @ x
    return y


def sim_rows(model, p, ds):
    """model reading per row = max of the model over the row window (t_k - 1, t_k]."""
    A, B, C, g = system(model, p)
    Ad, Bd = discretize(A, B, DT)
    u = np.repeat(ds["tau"], NSUB)                 # step i inside row k uses row k's mean torque
    y = run_lin(Ad, Bd, C, g(u)) + ds["T0"]         # grid t[0]-1 + i*DT
    win = y[1:].reshape(len(ds["tau"]), NSUB)
    return win.max(axis=1)


def resid(model, p, dss):
    out = []
    for ds in dss:
        r = sim_rows(model, p, ds) - ds["T"]
        out.append(r[~np.isnan(r)])
    return np.concatenate(out)


SPEC = {  # p0, lower, upper, names
    "M1": ([0.024, 16.0], [1e-4, 1.0], [1.0, 1000.0], ["a_C_per_s_Nm2", "tau_th_s"]),
    "M2": ([0.02, 0.01, 16.0], [0.0, 0.0, 1.0], [1.0, 1.0, 1000.0], ["a_C_per_s_Nm2", "b_C_per_s_Nm", "tau_th_s"]),
    "M2b": ([0.15, 16.0], [0.0, 1.0], [5.0, 1000.0], ["b_C_per_s_Nm", "tau_th_s"]),
    "M3": ([0.024, 15.0, 50.0, 900.0], [1e-4, 0.5, 0.5, 5.0], [1.0, 500.0, 1e4, 1e5], ["a_C_per_s_Nm2", "tau_w_s", "r_Ch_over_Cw", "tau_h_s"]),
    "M4": ([0.025, 15.0, 1.0], [1e-4, 1.0, 0.05], [1.0, 1000.0, 60.0], ["a_C_per_s_Nm2", "tau_th_s", "tau_s_s"]),
}
M3B = dict(r=20.0, th=1800.0)   # housing bound, chosen in section 3c


def fit(model, dss, fixed=None):
    p0, lo, hi, names = SPEC[model]
    if fixed:   # M3 with r, th fixed
        f = lambda q: resid("M3", [q[0], q[1], fixed["r"], fixed["th"]], dss)
        res = least_squares(f, [0.024, 15.0], bounds=([1e-4, 0.5], [1.0, 500.0]), x_scale="jac")
        p = [res.x[0], res.x[1], fixed["r"], fixed["th"]]
    else:
        res = least_squares(lambda q: resid(model, q, dss), p0, bounds=(lo, hi), x_scale="jac")
        p = list(res.x)
    n_par = 2 if fixed else len(p0)
    rms = float(np.sqrt(np.mean(res.fun ** 2)))
    # formal standard errors (optimistic: residuals are quantized and correlated)
    try:
        J = res.jac; s2 = np.sum(res.fun ** 2) / max(1, len(res.fun) - n_par)
        se = list(np.sqrt(np.diag(np.linalg.pinv(J.T @ J) * s2)))
    except Exception:
        se = [float("nan")] * n_par
    return dict(p=[float(v) for v in p], rms=rms, n=len(res.fun), se=[float(v) for v in se])


def mname(model, fixed):
    return "M3B" if fixed else model


def rms_on(model, p, dss):
    r = resid(model, p, dss)
    return float(np.sqrt(np.mean(r ** 2)))


DS = {(n, s): make_ds(n, s) for n in ("RT6b", "RT6c") for s in "LR"}
fits = []        # every fit, for the table and JSON
FIT = {}
for model, fixed in (("M1", None), ("M2", None), ("M2b", None), ("M3", None), ("M4", None), ("M3", M3B)):
    key = mname(model, fixed)
    base = "M3" if fixed else model
    for s in "LR":
        for train, test in ((("RT6b", "RT6c"), ()), (("RT6b",), ("RT6c",)), (("RT6c",), ("RT6b",))):
            fr = fit(base, [DS[(n, s)] for n in train], fixed)
            row = dict(model=key, side=s, train="+".join(train), test="+".join(test) if test else "",
                       rms_train_C=fr["rms"], n_train=fr["n"],
                       rms_test_C=rms_on(base, fr["p"], [DS[(n, s)] for n in test]) if test else float("nan"))
            for nm, v, se in zip(SPEC[base][3], fr["p"], fr["se"] + [float("nan")] * 4):
                row[nm] = v; row[nm + "_se"] = se
            for n in ("RT6b", "RT6c"):
                row["rms_" + n + "_C"] = rms_on(base, fr["p"], [DS[(n, s)]])
            fits.append(row); FIT[(key, s, "+".join(train))] = fr
    if model == "M1":   # both shoulders, one parameter set
        fr = fit("M1", list(DS.values()))
        row = dict(model="M1", side="L+R", train="RT6b+RT6c", test="", rms_train_C=fr["rms"], n_train=fr["n"], rms_test_C=float("nan"))
        for nm, v, se in zip(SPEC["M1"][3], fr["p"], fr["se"]):
            row[nm] = v; row[nm + "_se"] = se
        fits.append(row); FIT[("M1", "L+R", "RT6b+RT6c")] = fr
write_csv(OUT + "/model_fits.csv", fits)

# heating-only M1 fit (rows up to [q]) to compare the heating time constant with cooling
heat_only = {}
for s in "LR":
    dss = [make_ds(n, s, t_to=EV[n]["q"] + 0.05) for n in ("RT6b", "RT6c")]
    fr = fit("M1", dss); heat_only[s] = fr

# T0 sensitivity (+-0.5 C) and timing sensitivity (+-0.5 s) for M1, left, both runs
sens = []
for dT0 in (-0.5, 0.5):
    dss = [dict(DS[(n, "L")], T0=DS[(n, "L")]["T0"] + dT0) for n in ("RT6b", "RT6c")]
    fr = fit("M1", dss); sens.append(dict(change=f"T0 {dT0:+.1f} C", a=fr["p"][0], tau_th=fr["p"][1], rms=fr["rms"]))
for dt_shift in (-0.5, 0.5):
    dss = []
    for n in ("RT6b", "RT6c"):
        ds = DS[(n, "L")]
        # shift torque relative to temperature by dt_shift s: resample torque on shifted grid
        tq = np.interp(ds["t"] - dt_shift, ds["t"], ds["tau"])
        dss.append(dict(ds, tau=tq))
    fr = fit("M1", dss); sens.append(dict(change=f"torque shifted {dt_shift:+.1f} s", a=fr["p"][0], tau_th=fr["p"][1], rms=fr["rms"]))

# 3c. housing profile: r x tau_h, a and tau_w refitted; spot readings as a check
prof = []
for s in "LR":
    dss = [DS[("RT6b", s)], DS[("RT6c", s)]]
    for th in (600.0, 1800.0):
        for r in (5.0, 10.0, 20.0, 40.0, 100.0, 1e4):
            fr = fit("M3", dss, dict(r=r, th=th))
            row = dict(side=s, tau_h_s=th, r=r, a=fr["p"][0], tau_w_s=fr["p"][1], rms_C=fr["rms"],
                       ss_C_per_Nm2=fr["p"][0] * (fr["p"][1] + th / r))
            row["tau_indef_Nm_10C"] = math.sqrt(10 / row["ss_C_per_Nm2"])
            for (rn, ss_), (tt, v, src) in SPOT.items():
                if ss_ == s:
                    y = sim_rows("M3", fr["p"], make_ds(rn, s, extend_to=tt))[-1]
                    row[f"spot_{rn}_pred_C"] = float(y); row[f"spot_{rn}_obs_C"] = v
            prof.append(row)
write_csv(OUT + "/housing_profile.csv", prof)

# ----------------------------------------------------------------------------- 2. rate against torque
M1L = FIT[("M1", "L", "RT6b+RT6c")]["p"]; M1R = FIT[("M1", "R", "RT6b+RT6c")]["p"]
TAU_TH = {"L": M1L[1], "R": M1R[1]}
A_FIT = {"L": M1L[0], "R": M1R[0]}


def slope(t, T):
    ols = np.polyfit(t - t.mean(), T, 1)[0]
    ts = stats.theilslopes(T, t)[0]
    return float(ols), float(ts)


win_rows, block_rows = [], []
NW = 5
for n in ("RT6b", "RT6c"):
    d = W[n]; e = EV[n]
    idx = np.where((d["t"] - 1.0 >= e["follow"]) & (d["t"] <= e["q"] + 0.05))[0]
    for s in "LR":
        T = d["T" + s]; tau = d["tau" + s]
        for start in range(idx[0], idx[-1] - NW + 2):
            ii = np.arange(start, start + NW)
            o, th_ = slope(d["t"][ii], T[ii])
            Tm = T[ii].mean()
            r = dict(run=n, side=s, rows=f"{d['rows'][ii[0]]['time']} to {d['rows'][ii[-1]]['time']}",
                     t_start=hms(d["t"][ii[0]] - 1)[:8], t_end=hms(d["t"][ii[-1]]), T_first=int(T[ii[0]]), T_last=int(T[ii[-1]]),
                     tau_abs_mean_Nm=float(np.mean(np.abs(tau[ii]))), tau2_mean_Nm2=float(np.mean(tau[ii] ** 2)),
                     rate_ols_C_s=o, rate_theilsen_C_s=th_, T_mean_C=float(Tm),
                     rate_loss_corrected_C_s=o + (Tm - T0[(n, s)]) / TAU_TH[s],
                     nonoverlap=int((start - idx[0]) % NW == 0 and start + NW - 1 <= idx[-1]))
            win_rows.append(r)
            if r["nonoverlap"]:
                block_rows.append(r)
# hold / damping phases: whole-phase OLS slope
hold_rows = []
for n, lab, mfun in (
        ("RT6b", "damping before launch", lambda d, e: (d["kp"] == 0) & (d["t"] < e["kp_on"])),
        ("RT6b", "hold before [r], kp 80", lambda d, e: (d["kp"] == 80) & (d["t"] < e["r"])),
        ("RT6c", "hold before [r], kp 80", lambda d, e: (d["kp"] == 80) & (d["t"] < e["r_first"]))):
    d = W[n]; e = EV[n]; m = mfun(d, e)
    for s in "LR":
        o, th_ = slope(d["t"][m], d["T" + s][m])
        hold_rows.append(dict(run=n, side=s, phase=lab, t_start=hms(d["t"][m][0] - 1)[:8], t_end=hms(d["t"][m][-1])[:8],
                              duration_s=float(d["t"][m][-1] - d["t"][m][0] + 1), T_first=int(d["T" + s][m][0]),
                              T_last=int(d["T" + s][m][-1]), tau_abs_mean_Nm=float(np.mean(np.abs(d["tau" + s][m]))),
                              tau2_mean_Nm2=float(np.mean(d["tau" + s][m] ** 2)), rate_ols_C_s=o, rate_theilsen_C_s=th_))
write_csv(OUT + "/rate_windows.csv", win_rows)
write_csv(OUT + "/rate_blocks.csv", block_rows)
write_csv(OUT + "/rate_holds.csv", hold_rows)

# through-origin fits of the raw rate: rate = k*|tau| vs rate = k*tau^2 (sliding windows)
def origin_fit(x, y):
    k = float(np.sum(x * y) / np.sum(x * x)); res = y - k * x
    return k, float(1 - np.sum(res ** 2) / np.sum((y - y.mean()) ** 2)), float(np.sqrt(np.mean(res ** 2)))


law = {}
for s in "LR":
    ww = [r for r in win_rows if r["side"] == s]
    x1 = np.array([r["tau_abs_mean_Nm"] for r in ww]); x2 = np.array([r["tau2_mean_Nm2"] for r in ww])
    y = np.array([r["rate_ols_C_s"] for r in ww]); yc = np.array([r["rate_loss_corrected_C_s"] for r in ww])
    law[s] = dict(raw_abs=origin_fit(x1, y), raw_sq=origin_fit(x2, y), corr_abs=origin_fit(x1, yc), corr_sq=origin_fit(x2, yc),
                  n=len(ww))
    # power law on the loss-corrected rate
    mpos = yc > 0.05
    law[s]["power_exponent"] = float(np.polyfit(np.log(x1[mpos]), np.log(yc[mpos]), 1)[0])

fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.4))
for s, col in (("L", C_L), ("R", C_R)):
    for n, mk in (("RT6b", "o"), ("RT6c", "s")):
        ww = [r for r in win_rows if r["side"] == s and r["run"] == n]
        x1 = [r["tau_abs_mean_Nm"] for r in ww]; x2 = [r["tau2_mean_Nm2"] for r in ww]
        y = [r["rate_ols_C_s"] for r in ww]; yc = [r["rate_loss_corrected_C_s"] for r in ww]
        lab = f"{'left' if s == 'L' else 'right'}, {n}"
        axs[0].plot(x1, y, mk, ms=6, color=col, alpha=0.85, mec=SURF, mew=1, label=lab)
        axs[1].plot(x2, y, mk, ms=6, color=col, alpha=0.85, mec=SURF, mew=1, label=lab + ", raw")
        axs[1].plot(x2, yc, mk, ms=6, mfc="none", color=col, alpha=0.85, label=lab + ", plus loss term")
    for h in hold_rows:
        if h["side"] == s:
            axs[0].plot(h["tau_abs_mean_Nm"], h["rate_ols_C_s"], "^", ms=8, color=col, mec=INK, mew=0.6)
            axs[1].plot(h["tau2_mean_Nm2"], h["rate_ols_C_s"], "^", ms=8, color=col, mec=INK, mew=0.6)
xx = np.linspace(0, 75, 50)
axs[1].plot(xx, A_FIT["L"] * xx, color=INK2, lw=1.2, ls="--")
axs[1].annotate(f"model heat input a*tau^2, a = {A_FIT['L']:.4f} C/s per Nm^2", (60, A_FIT["L"] * 60), xytext=(-210, 30),
                textcoords="offset points", fontsize=8, color=INK2, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.6))
axs[0].set_xlabel("mean |tau_est| over the 5 s window (Nm)"); axs[0].set_ylabel("temperature rate, 5-row OLS slope (C/s)")
axs[1].set_xlabel("mean tau_est^2 over the window (Nm^2)")
axs[0].set_title("rate against |torque| (triangles: holds before [r], about 0.1 Nm)", loc="left", fontsize=9.5)
axs[1].set_title("rate against torque squared (hollow: rate + (T - T0)/tau_th)", loc="left", fontsize=9.5)
axs[0].legend(fontsize=7.5, loc="upper left"); axs[1].legend(fontsize=7, loc="upper left", ncol=1)
for ax in axs:
    ax.axhline(0, color=INK2, lw=0.8)
fig.tight_layout()
savefig(fig, "fig2_rate_vs_torque.png")

# ----------------------------------------------------------------------------- 3b. RT6 check (no torque logged)
d6 = W["RT6"]; e6 = EV["RT6"]
t_flat = sec("23:44:41.0")            # last second the hottest arm motor read 52 C with another joint hottest
t6 = d6["t"][(d6["t"] > t_flat)]
known = (d6["temp_joint"][(d6["t"] > t_flat)] == 15)
Tmax6 = d6["temp_max"][(d6["t"] > t_flat)]


def rt6_sim(model, p, tau_c, T_start, T0_6, t_off=None, x_extra=None):
    """constant torque from t_flat to exit, then 0; returns readings per RT6 row after t_flat, and a function for later times."""
    A, B, C, g = system(model, p)
    Ad, Bd = discretize(A, B, DT)
    t_end = (census["census_end.json"]["t_end"] + 1) if t_off is None else t_off
    grid = np.arange(t_flat, t_end + DT, DT)
    u = np.where(grid[:-1] < e6["exit"], tau_c, 0.0)
    nx = A.shape[0]
    x0 = np.zeros(nx); x0[:] = T_start - T0_6
    if model == "M3":
        x0[1] = 0.0             # housing at rest
    if model == "M4":
        x0[:] = T_start - T0_6
    y = run_lin(Ad, Bd, C, g(u), x0) + T0_6
    return grid, y


def rt6_readings(grid, y, tq):
    out = []
    for tk in tq:
        mm = (grid > tk - 1) & (grid <= tk + 1e-9)
        out.append(y[mm].max())
    return np.array(out)


rt6 = []
for model, key, p in (("M1", "M1", M1L), ("M4", "M4", FIT[("M4", "L", "RT6b+RT6c")]["p"]),
                      ("M3", "M3B", FIT[("M3B", "L", "RT6b+RT6c")]["p"])):
    for T0_6 in (49.0, 51.0):
        def res6(q):
            grid, y = rt6_sim(model, p, q[0], 52.0, T0_6)
            pr = rt6_readings(grid, y, t6)
            r = pr - Tmax6
            r = np.where(known, r, np.maximum(r, 0.0))      # where left is not the hottest: left <= arm max
            return r
        sol = least_squares(res6, [9.0], bounds=([0.0], [30.0]))
        grid, y = rt6_sim(model, p, sol.x[0], 52.0, T0_6)
        c = census["census_end.json"]
        cen = y[(grid > c["t_end"] - c["window_s"]) & (grid <= c["t_end"])].max()
        rt6.append(dict(model=key, T0_C=T0_6, tau_const_Nm=float(sol.x[0]), rms_C=float(np.sqrt(np.mean(sol.fun ** 2))),
                        n_rows=int(len(t6)), peak_model_C=float(y.max()), census_pred_C=float(cen), census_obs_C=c["L"]))
write_csv(OUT + "/rt6_check.csv", rt6)

# ----------------------------------------------------------------------------- 4. cooling
def cool_fit(t, T, t_s, T0f=None):
    """reading_k = max over (t_k-1, t_k] of T0 + A exp(-(t - t_s)/tau) = value at max(t_k - 1, t_s)."""
    te = np.maximum(t - 1.0, t_s) - t_s
    if T0f is None:
        f = lambda q: q[2] + q[0] * np.exp(-te / q[1]) - T
        res = least_squares(f, [T[0] - T.min() + 1, 15.0, T.min() - 0.5], bounds=([0, 1, 30], [80, 1000, 70]))
        A_, tau_, T0_ = res.x
    else:
        f = lambda q: T0f + q[0] * np.exp(-te / q[1]) - T
        res = least_squares(f, [T[0] - T0f, 15.0], bounds=([0, 1], [80, 1000]))
        (A_, tau_), T0_ = res.x, T0f
    J = res.jac; s2 = np.sum(res.fun ** 2) / max(1, len(T) - len(res.x))
    se = np.sqrt(np.diag(np.linalg.pinv(J.T @ J) * s2))
    return dict(A=float(A_), tau=float(tau_), tau_se=float(se[1]), T0=float(T0_), rms=float(np.sqrt(np.mean(res.fun ** 2))),
                n=len(T), t_first=hms(t[0])[:8], t_last=hms(t[-1])[:8])


cool = []
for n in ("RT6b", "RT6c"):
    d = W[n]; e = EV[n]
    m = d["t"] - 1.0 >= e["release"] - 1.0
    m &= d["t"] > e["release"] + 0.5
    for s in "LR":
        for T0f, lab in ((T0[(n, s)], "T0 fixed at baseline"), (None, "T0 free")):
            cf = cool_fit(d["t"][m], d["T" + s][m], e["release"], T0f)
            cf.update(run=n, side=s, variant=lab, start=f"release done {hms(e['release'])}")
            cool.append(cf)
# RT6: arms unpowered after exit; left is the hottest arm motor in every row after exit
m = (d6["t"] > e6["exit"] + 0.5) & (d6["temp_joint"] == 15)
for T0f, lab in ((51.0, "T0 fixed 51 C (assumed)"), (49.0, "T0 fixed 49 C (assumed)"), (None, "T0 free")):
    cf = cool_fit(d6["t"][m], d6["temp_max"][m], e6["exit"], T0f)
    cf.update(run="RT6", side="L", variant=lab, start=f"launcher exit {hms(e6['exit'])}")
    c = census["census_end.json"]
    tc = c["t_end"] - c["window_s"]           # census max = value at the window start (cooling)
    cf["census_pred_C"] = float(cf["T0"] + cf["A"] * math.exp(-(tc - e6["exit"]) / cf["tau"])); cf["census_obs_C"] = c["L"]
    cool.append(cf)
write_csv(OUT + "/cooling_fits.csv", cool)

fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.2), gridspec_kw=dict(width_ratios=[1.35, 1]))
ax = axs[0]
for n, s, col, mk in (("RT6", "L", INK2, "^"), ("RT6b", "L", C_L, "o"), ("RT6b", "R", C_R, "o"), ("RT6c", "L", C_L, "s"), ("RT6c", "R", C_R, "s")):
    d = W[n]; e = EV[n]
    if n == "RT6":
        m = (d["t"] > e["exit"] + 0.5) & (d["temp_joint"] == 15); T = d["temp_max"][m]
    else:
        m = d["t"] > e["release"] + 0.5; T = d["T" + s][m]
    x = d["t"][m] - 0.5 - e["release"]
    ax.plot(x, T, mk, ms=5, color=col, mfc=col if n != "RT6c" else "none", label=f"{n} {'left' if s == 'L' else 'right'}")
    cf = [c for c in cool if c["run"] == n and c["side"] == s and c["variant"].startswith("T0 fixed")][0]
    xx = np.linspace(0, 60, 200)
    ax.plot(xx, cf["T0"] + cf["A"] * np.exp(-xx / cf["tau"]), color=col, lw=1.2, ls="-" if n != "RT6c" else "--", alpha=0.9)
c = census["census_end.json"]
ax.plot([c["t_end"] - c["window_s"] - e6["exit"], c["t_end"] - e6["exit"]], [c["L"]] * 2, color=INK2, lw=3, solid_capstyle="butt")
ax.annotate("RT6 census (max over window)", (c["t_end"] - e6["exit"], c["L"]), xytext=(-10, 12), textcoords="offset points", fontsize=7.5, color=INK2, ha="right")
for (rn, s), (tt, v, src) in SPOT.items():
    xs = tt - EV[rn]["release"]
    if xs < 60:
        ax.plot([xs], [v], "D", ms=6, mfc="none", color=C_L if s == "L" else C_R)
ax.set_xlim(0, 60); ax.set_xlabel("seconds after release (RT6: after launcher exit)"); ax.set_ylabel("temperature reading (C)")
ax.set_title("fast cooling: readings and exponential fits (T0 fixed)", loc="left", fontsize=9.5)
ax.legend(fontsize=7.5)
ax = axs[1]
m6 = (d6["t"] > e6["exit"] + 0.5) & (d6["temp_joint"] == 15)
ax.plot((d6["t"][m6] - 0.5 - e6["exit"]) / 60, d6["temp_max"][m6], "^", ms=5, color=INK2, label="RT6 left, arm_watch")
for f, cc in census.items():
    if f != "census_start.json":
        ax.plot([(cc["t_end"] - e6["exit"]) / 60], [cc["L"]], "D", ms=6, color=C_L)
        ax.plot([(cc["t_end"] - e6["exit"]) / 60], [cc["R"]], "D", ms=6, color=C_R)
ax.plot([(r["t"] - e6["exit"]) / 60 for r in cool_log], [r["L"] for r in cool_log], "o", ms=5, color=C_L, label="left, census / cool-down log")
ax.plot([(r["t"] - e6["exit"]) / 60 for r in cool_log], [r["R"] for r in cool_log], "o", ms=5, color=C_R, label="right, census / cool-down log")
ax.set_xlabel("minutes after RT6 launcher exit (23:45:08)"); ax.set_ylabel("temperature reading (C)")
ax.set_title("slow tail after RT6 (robot rebooted before 00:05)", loc="left", fontsize=9.5)
ax.legend(fontsize=7.5)
fig.tight_layout()
savefig(fig, "fig4_cooling.png")

# ----------------------------------------------------------------------------- 3. model fit figure
fig, axs = plt.subplots(2, 2, figsize=(11.5, 6.6), sharex="row")
for i, n in enumerate(("RT6b", "RT6c")):
    other = "RT6c" if n == "RT6b" else "RT6b"
    for j, s in enumerate("LR"):
        ax = axs[i, j]; ds = DS[(n, s)]; z = EV[n]["r"]
        shade(ax, n)
        x = ds["t"] - 0.5 - z
        ax.plot(x, ds["T"], "o", ms=4, color=C_L if s == "L" else C_R, label="reading (integer)")
        y1 = sim_rows("M1", FIT[("M1", s, "RT6b+RT6c")]["p"], ds)
        y2 = sim_rows("M1", FIT[("M1", s, other)]["p"], ds)
        y3 = sim_rows("M3", FIT[("M3B", s, "RT6b+RT6c")]["p"], ds)
        ax.plot(x, y1, color=INK, lw=1.6, label=f"M1 fitted on both runs, RMS {rms_on('M1', FIT[('M1', s, 'RT6b+RT6c')]['p'], [ds]):.2f} C")
        ax.plot(x, y2, color=C_3, lw=1.6, ls="--", label=f"M1 fitted on {other} only, RMS {rms_on('M1', FIT[('M1', s, other)]['p'], [ds]):.2f} C")
        ax.plot(x, y3, color=INK2, lw=1.0, ls=":", label=f"M3 with housing bound, RMS {rms_on('M3', FIT[('M3B', s, 'RT6b+RT6c')]['p'], [ds]):.2f} C")
        ax.set_xlim(-10, ds["t"][-1] - z + 1)
        ax.set_title(f"{n}, {'left' if s == 'L' else 'right'} shoulder pitch", loc="left", fontsize=9.5)
        ax.legend(fontsize=7, loc="upper left")
        if j == 0:
            ax.set_ylabel("temperature (C)")
        if i == 1:
            ax.set_xlabel("seconds from [r] (shaded: start/blend, FOLLOW, lowering)")
fig.tight_layout()
savefig(fig, "fig3_model_fit.png")

# ----------------------------------------------------------------------------- 5. predictions
TORQ = [2.0, 2.5, 3.0, 4.0, 6.0, 8.0]
T_START, T_LIM = 50.0, 60.0


def pred_curve(model, p, tau, t_max, dt=0.1):
    A, B, C, g = system(model, p)
    Ad, Bd = discretize(A, B, dt)
    n = int(t_max / dt)
    y = run_lin(Ad, Bd, C, g(np.full(n, tau))) + T_START
    return np.arange(n + 1) * dt, y


def ss_coef(model, p):
    if model in ("M1", "M4"):
        return p[0] * p[1]
    if model == "M3":
        return p[0] * (p[1] + p[3] / p[2])


def t_to_lim(model, p, tau, t_max=7200.0):
    if T_START + ss_coef(model, p) * tau ** 2 <= T_LIM:
        return float("inf")
    if model == "M1":
        a, tt = p
        return -tt * math.log(1 - (T_LIM - T_START) / (a * tau ** 2 * tt))
    t, y = pred_curve(model, p, tau, t_max, dt=0.05 if tau > 3 else 0.5)
    k = np.argmax(y >= T_LIM)
    return float(t[k]) if y[k] >= T_LIM else float("inf")


def max_tau(model, p, D):
    if D == float("inf"):
        return math.sqrt((T_LIM - T_START) / ss_coef(model, p))
    f = lambda tau: (t_to_lim(model, p, tau) - D)
    lo, hi = math.sqrt((T_LIM - T_START) / ss_coef(model, p)), 20.0
    if f(lo + 1e-6) < 0:
        return lo
    return brentq(lambda tau: min(t_to_lim(model, p, tau), 1e6) - D, lo + 1e-6, hi, xtol=1e-3)


PM = {  # prediction model sets
    "M1 left, both runs": ("M1", FIT[("M1", "L", "RT6b+RT6c")]["p"]),
    "M1 right, both runs": ("M1", FIT[("M1", "R", "RT6b+RT6c")]["p"]),
    "M1 left, RT6b only": ("M1", FIT[("M1", "L", "RT6b")]["p"]),
    "M1 left, RT6c only": ("M1", FIT[("M1", "L", "RT6c")]["p"]),
    "M1 right, RT6b only": ("M1", FIT[("M1", "R", "RT6b")]["p"]),
    "M1 right, RT6c only": ("M1", FIT[("M1", "R", "RT6c")]["p"]),
    "M4 left, both runs": ("M4", FIT[("M4", "L", "RT6b+RT6c")]["p"]),
    "M3 housing bound, left": ("M3", FIT[("M3B", "L", "RT6b+RT6c")]["p"]),
    "M3 housing bound, right": ("M3", FIT[("M3B", "R", "RT6b+RT6c")]["p"]),
}
pred = []
for lab, (model, p) in PM.items():
    row = dict(model_set=lab)
    for tau in TORQ:
        row[f"t60_{tau:g}Nm_s"] = t_to_lim(model, p, tau)
        row[f"Tss_{tau:g}Nm_C"] = T_START + ss_coef(model, p) * tau ** 2
    for D, nm in ((120.0, "2min"), (600.0, "10min"), (float("inf"), "indef")):
        row[f"tau_max_{nm}_Nm"] = max_tau(model, p, D)
    pred.append(row)
write_csv(OUT + "/predictions.csv", pred)

fig, ax = plt.subplots(figsize=(8.6, 4.6))
cols = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#eb6834", "#c94a1c"]
for tau, col in zip(TORQ, cols):
    t, y = pred_curve("M1", PM["M1 left, both runs"][1], tau, 3600, dt=0.1)
    ax.plot(t[1:], y[1:], color=col, lw=2, label=f"{tau:g} Nm")
    t, y = pred_curve("M3", PM["M3 housing bound, left"][1], tau, 3600, dt=0.5)
    ax.plot(t[1:], y[1:], color=col, lw=1.2, ls="--")
ax.axhline(60, color=INK2, lw=1, ls="--"); ax.text(1.1, 60.6, "60 C project stop", fontsize=8, color=INK2)
for D, lab in ((120, "2 min"), (600, "10 min")):
    ax.axvline(D, color=INK2, lw=0.8, ls=":"); ax.text(D * 1.05, 51, lab, fontsize=8, color=INK2)
ax.set_xscale("log"); ax.set_xlim(1, 3600); ax.set_ylim(49, 80)
ax.set_xlabel("seconds at constant torque, from 50 C at rest (log scale)"); ax.set_ylabel("predicted reading (C)")
ax.set_title("left shoulder pitch: M1 (solid) and M3 with the housing bound (dashed)", loc="left", fontsize=9.5)
ax.legend(title="constant torque", fontsize=8, title_fontsize=8, loc="upper left")
fig.tight_layout()
savefig(fig, "fig5_predictions.png")

# ----------------------------------------------------------------------------- claims check
claims = []
dB, dC = W["RT6b"], W["RT6c"]
def seg(d, s, t0s, t1s):
    m = (d["t"] - 0.5 >= sec(t0s)) & (d["t"] - 0.5 <= sec(t1s))
    return d["t"][m], d["T" + s][m], d["tau" + s][m]
for lab, d, s, a_, b_ in (("about 2.5 Nm, RT6b left", dB, "L", "00:25:48.5", "00:25:56.5"),
                          ("about 4 Nm, RT6c left", dC, "L", "00:38:22.6", "00:38:28.6"),
                          ("6 to 8.5 Nm, RT6b left", dB, "L", "00:25:59.5", "00:26:07.5"),
                          ("6 to 8.5 Nm, RT6c left", dC, "L", "00:38:32.6", "00:38:39.6")):
    t, T, tq = seg(d, s, a_, b_)
    o, th_ = slope(t, T)
    claims.append(dict(claim=lab, window=f"{hms(t[0] - 1)[:8]} to {hms(t[-1])[:8]}", n_rows=len(t),
                       tau_range=f"{np.abs(tq).min():.2f} to {np.abs(tq).max():.2f}", T=f"{int(T[0])} to {int(T[-1])}",
                       rate_ols_C_s=o, rate_theilsen_C_s=th_, span_s=float(t[-1] - t[0])))
# cooling claim: 63 C to 52 C
k63 = dB["k_stop"]; k52 = int(np.argmax((dB["TL"] <= 52) & (np.arange(len(dB["t"])) > k63)))
claims.append(dict(claim="63 to 52 C after [q], RT6b left", window=f"{dB['rows'][k63]['time']} to {dB['rows'][k52]['time']}",
                   n_rows=k52 - k63 + 1, tau_range="", T=f"{int(dB['TL'][k63])} to {int(dB['TL'][k52])}",
                   rate_ols_C_s=float("nan"), rate_theilsen_C_s=float("nan"), span_s=float(dB["t"][k52] - dB["t"][k63])))

# ----------------------------------------------------------------------------- summary JSON
summary = dict(
    note="RT6d heat analysis; all temperatures are integer readings in C (max of the motor's two values over ~1 s windows)",
    row_time_calibration={n: dict(stop_row_label=W[n]["rows"][W[n]["k_stop"]]["time"], q_time=hms(EV[n]["q"]),
                                  label_minus_floor_time=W[n]["label_check"]) for n in W},
    T0_baseline_C={f"{n}_{s}": v for (n, s), v in T0.items()},
    rest=rest, census=census, cooldown_log=cool_log,
    before_restart=dict(rows=len(br), arm_max_C=[min(br_max), max(br_max)], hottest_joints=br_joint),
    fits=fits, heating_only_M1={s: v for s, v in heat_only.items()}, sensitivity_M1_left=sens,
    housing_profile=prof, housing_bound=M3B, rate_law=law, holds=hold_rows, rate_blocks=block_rows,
    rt6_check=rt6, cooling=cool, predictions=pred, claims=claims, spot_readings={f"{k[0]}_{k[1]}": dict(time=hms(v[0])[:8], C=v[1], source=v[2]) for k, v in SPOT.items()},
)
json.dump(summary, open(OUT + "/summary.json", "w"), indent=1, default=lambda o: o if not isinstance(o, float) else round(o, 5))


# ----------------------------------------------------------------------------- console digest
def p(*a):
    print(*a)


p("row time calibration (label - floor(t) should be 0 or 1):", {n: W[n]["label_check"] for n in W})
p("T0:", T0)
for r in fits:
    if r["test"] == "" or r["model"] == "M1":
        keys = [k for k in r if k.endswith("_s") or k.startswith("a_") or k.startswith("b_") or k.startswith("r_")]
        p(f"{r['model']:4s} {r['side']:3s} train {r['train']:10s} test {r['test']:5s} rms_train {r['rms_train_C']:.3f} rms_test {r['rms_test_C']:.3f}",
          {k: round(r[k], 4) for k in keys if not k.endswith("_se")})
p("heating-only M1:", {s: (round(v["p"][0], 4), round(v["p"][1], 1), round(v["rms"], 3)) for s, v in heat_only.items()})
p("sensitivity:", [(x["change"], round(x["a"], 4), round(x["tau_th"], 1), round(x["rms"], 3)) for x in sens])
p("law:", json.dumps(law, default=lambda o: round(o, 4)))
p("holds:", [(h["run"], h["side"], h["phase"], h["T_first"], h["T_last"], round(h["duration_s"]), round(h["rate_ols_C_s"], 5)) for h in hold_rows])
p("blocks:")
for b in block_rows:
    p(f"  {b['run']} {b['side']} {b['t_start']}-{b['t_end'][:8]} |tau| {b['tau_abs_mean_Nm']:.2f} tau2 {b['tau2_mean_Nm2']:.1f} T {b['T_first']}->{b['T_last']} "
      f"ols {b['rate_ols_C_s']:.3f} ts {b['rate_theilsen_C_s']:.3f} corr {b['rate_loss_corrected_C_s']:.3f}")
p("profile:")
for r in prof:
    p("  ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
p("rt6 check:", [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()} for r in rt6])
p("cooling:")
for c in cool:
    p("  ", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in c.items()})
p("claims:", [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in c.items()} for c in claims])
p("predictions:")
for r in pred:
    p("  ", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})
p("rest:", rest)
p("census:", census); p("before_restart:", summary["before_restart"])
