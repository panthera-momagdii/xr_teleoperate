#!/usr/bin/env python3
"""G16 (RT7): the arm watcher's temperature rule, housing [0] and winding [1] separately.
DOMAIN 1, LOOPBACK ONLY. No launcher: the watcher's [q] goes to a dummy tmux pane that
records the keys it receives.

Rule (Mohammed, 23 Oct; LowState temperature[0] = housing, [1] = winding): housing WARN 75 C,
STOP 80 C; winding WARN 110 C, STOP 120 C; STOP = 2 consecutive seconds at or over a stop
value, then [q] once. Stop values above 80 / 120 C and the old single-value env refuse.

  (A) housing rises 74 C + 1 C/s, winding 60 C: WARN housing at 75, STOP housing at 80 after
      2 consecutive seconds, one [q], no winding WARN; the CSV keeps the two values apart.
  (B) winding rises 116 C + 1 C/s, housing 50 C: WARN winding, STOP winding at 120, one [q].
  (C) housing 79 C and winding 119 C, steady: both WARN, no STOP, no [q].
  (D) a lower stop value is honoured (housing stop 70 C).
  (E) refusals: ARM_WATCH_STOP_C (old), housing stop 85 C, winding stop 125 C.

Run:  python tools/overnight/test_g16_watch_temps.py
"""
import csv, os, signal, subprocess, sys, tempfile, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
PY = sys.executable
WATCH = Path("/home/unitree/panthera/remote-teleop-20261012/rt6/tools/arm_watch.py")
results = []


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


tmp = Path(tempfile.mkdtemp(prefix="g16_"))
BASE = {k: v for k, v in os.environ.items() if not k.startswith("ARM_WATCH_")}
BASE.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")


def case(tag, fake_args, seconds, env_extra=None):
    keys = tmp / f"keys_{tag}.txt"
    keys.write_text("")
    subprocess.run(["tmux", "kill-session", "-t", "g16w"], capture_output=True)
    subprocess.run(["tmux", "new", "-d", "-s", "g16w", "-x", "80", "-y", "10",
                    f"bash -c 'stty -icanon -echo min 1; cat > {keys}'"], check=True)
    fx = subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo", "--rate", "500",
                           "--q-arms", "0.0"] + fake_args, env=BASE, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(1.5)
    out_csv = tmp / f"watch_{tag}.csv"
    out_txt = tmp / f"watch_{tag}.out"
    w = subprocess.Popen([PY, "-I", "-B", str(WATCH), str(out_csv), "g16w", "1", "lo"],
                         env=dict(BASE, **(env_extra or {})), stdout=open(out_txt, "w"),
                         stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(seconds)
    stop(w)
    stop(fx)
    time.sleep(0.5)
    subprocess.run(["tmux", "kill-session", "-t", "g16w"], capture_output=True)
    rows = list(csv.DictReader(open(out_csv))) if out_csv.exists() else []
    return dict(rows=rows, out=out_txt.read_text(errors="replace"), keys=keys.read_text(errors="replace"))


def notes(rows):
    return [(k, r["note"]) for k, r in enumerate(rows) if r.get("note")]


# ---------------------------------------------------------------- (A) housing
print("=== (A) housing [0] rises from 74 C at 1 C/s, winding [1] at 60 C ===", flush=True)
A = case("A", ["--temp-arms", "74", "--temp-rise", "1.0", "--temp-winding", "60"], 13)
print("   " + A["out"].strip().replace("\n", "\n   "))
record("A: thresholds line names both values", "housing [0] warn 75 C, stop 80 C; winding [1] warn 110 C, stop 120 C" in A["out"])
need = ["housing_max_C", "housing_joint", "winding_max_C", "winding_joint", "spL_housing_C", "spL_winding_C",
        "spR_housing_C", "spR_winding_C", "dqmax_15", "dqmax_28", "wrist_rev_19", "wrist_rev_28"]
record("A: CSV has the housing / winding, per-joint speed and wrist reversal columns",
       A["rows"] and all(c in A["rows"][0] for c in need), ", ".join(c for c in need if A["rows"] and c not in A["rows"][0]))
hv = [int(r["housing_max_C"]) for r in A["rows"]]; wv = [int(r["winding_max_C"]) for r in A["rows"]]
record("A: the CSV reads housing rising and winding at 60 C, apart", hv and max(hv) >= 80 and set(wv) == {60},
       f"housing {min(hv) if hv else '-'} to {max(hv) if hv else '-'} C, winding {sorted(set(wv))}")
record("A: shoulder pitch columns read [0] and [1] apart",
       A["rows"] and int(A["rows"][-1]["spL_winding_C"]) == 60 and int(A["rows"][-1]["spL_housing_C"]) >= 80,
       f"last row L {A['rows'][-1]['spL_housing_C']} / {A['rows'][-1]['spL_winding_C']} C" if A["rows"] else "")
na = notes(A["rows"])
wh = [k for k, n in na if "WARN temp housing [0] 7" in n]
st = [k for k, n in na if "STOP temp housing [0] 8" in n and "sent q" in n]
record("A: WARN housing at 75 C", bool(wh), na[0][1] if na else "no note")
record("A: no winding WARN", not any("winding" in n for _, n in na))
first80 = next((k for k, v in enumerate(hv) if v >= 80), None)
record("A: STOP housing only on the 2nd consecutive second at 80 C or more",
       bool(st) and first80 is not None and st[0] == first80 + 1,
       f"first second at >= 80: row {first80}, STOP row {st[0] if st else None}")
record("A: exactly one [q] reached the pane", A["keys"].count("q") == 1, f"keys {A['keys']!r}")

# ---------------------------------------------------------------- (B) winding
print("\n=== (B) winding [1] rises from 116 C at 1 C/s, housing [0] at 50 C ===", flush=True)
B = case("B", ["--temp-arms", "50", "--temp-winding", "116", "--temp-winding-rise", "1.0"], 10)
nb = notes(B["rows"])
record("B: WARN winding at 110 C or more", any("WARN temp winding [1] 11" in n for _, n in nb), nb[0][1] if nb else "no note")
record("B: no housing WARN", not any("housing" in n for _, n in nb))
sb = [k for k, n in nb if "STOP temp winding [1] 12" in n and "sent q" in n]
wv = [int(r["winding_max_C"]) for r in B["rows"]]
f120 = next((k for k, v in enumerate(wv) if v >= 120), None)
record("B: STOP winding on the 2nd consecutive second at 120 C or more",
       bool(sb) and f120 is not None and sb[0] == f120 + 1, f"first >= 120: row {f120}, STOP row {sb[0] if sb else None}")
record("B: exactly one [q]", B["keys"].count("q") == 1, f"keys {B['keys']!r}")

# ---------------------------------------------------------------- (C) just under
print("\n=== (C) housing 79 C, winding 119 C, steady ===", flush=True)
Cc = case("C", ["--temp-arms", "79", "--temp-winding", "119"], 8)
nc = notes(Cc["rows"])
record("C: both WARN", any("housing [0] 79" in n for _, n in nc) and any("winding [1] 119" in n for _, n in nc),
       "; ".join(n for _, n in nc))
record("C: no STOP and no [q] just under the stop values", not any("STOP" in n for _, n in nc) and "q" not in Cc["keys"],
       f"{len(Cc['rows'])} rows, keys {Cc['keys']!r}")

# ---------------------------------------------------------------- (D) lower stop honoured
print("\n=== (D) housing stop lowered to 70 C ===", flush=True)
D = case("D", ["--temp-arms", "68", "--temp-rise", "1.0", "--temp-winding", "60"], 9,
         {"ARM_WATCH_HOUSING_WARN_C": "65", "ARM_WATCH_HOUSING_STOP_C": "70"})
nd = notes(D["rows"])
record("D: a lower housing stop (70 C) is honoured", any("STOP temp housing [0] 7" in n for _, n in nd) and D["keys"].count("q") == 1,
       "; ".join(n for _, n in nd)[:160])

# ---------------------------------------------------------------- (E) refusals
print("\n=== (E) refusals ===", flush=True)
for tag, env in (("old ARM_WATCH_STOP_C", {"ARM_WATCH_STOP_C": "60"}),
                 ("housing stop 85 C", {"ARM_WATCH_HOUSING_STOP_C": "85"}),
                 ("winding stop 125 C", {"ARM_WATCH_WINDING_STOP_C": "125"}),
                 ("housing warn above stop", {"ARM_WATCH_HOUSING_WARN_C": "81"})):
    r = subprocess.run([PY, "-I", "-B", str(WATCH), str(tmp / "refuse.csv"), "g16w", "1", "lo"], env=dict(BASE, **env),
                       capture_output=True, text=True, timeout=30)
    record(f"E: refuses {tag}", r.returncode != 0 and "REFUSED" in (r.stdout + r.stderr),
           (r.stdout + r.stderr).strip().splitlines()[-1][:120] if (r.stdout + r.stderr).strip() else f"rc {r.returncode}")

print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, d in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {d}" if d else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed", flush=True)
sys.stdout.flush()
os._exit(1 if n_fail else 0)
