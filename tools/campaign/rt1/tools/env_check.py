#!/usr/bin/env python3
"""RT1 step 4e: env invariants for env xrt. Imports only; no DDS, no network."""
import glob, importlib, importlib.metadata as md, os, subprocess, sys
ok_all = True
def check(label, cond, detail):
    global ok_all
    ok_all &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:38s} {detail}")
def ver(dist):
    try: return md.version(dist)
    except md.PackageNotFoundError: return None
check("python 3.10", sys.version_info[:2] == (3, 10), sys.version.split()[0])
import pinocchio
conda_line = subprocess.run([os.path.expanduser("~/miniconda3/bin/conda"), "list", "-n", "xrt", "^pinocchio$"],
                            capture_output=True, text=True).stdout.strip().splitlines()[-1]
check("pinocchio 3.1.0 conda-forge", pinocchio.__version__ == "3.1.0" and conda_line.split()[-1] == "conda-forge",
      f"{pinocchio.__version__} | {' '.join(conda_line.split())}")
try:
    from pinocchio import casadi as cpin
    check("from pinocchio import casadi", True, cpin.__name__)
except Exception as e:
    check("from pinocchio import casadi", False, repr(e))
pin_show = subprocess.run([sys.executable, "-m", "pip", "show", "pin"], capture_output=True, text=True)
check("pip show pin finds nothing", pin_show.returncode != 0 and not pin_show.stdout.strip(),
      f"rc={pin_show.returncode} {pin_show.stderr.strip()[:60]}")
cmeel = glob.glob(os.path.join(sys.prefix, "lib/python3.10/site-packages/cmeel*"))
check("no cmeel prefix", not cmeel, cmeel or "none")
import cyclonedds, numpy, vuer, params_proto
check("cyclonedds 0.10.2", ver("cyclonedds") == "0.10.2", ver("cyclonedds"))
check("numpy 1.26.4", numpy.__version__ == "1.26.4", f"{numpy.__version__} {numpy.__file__}")
check("vuer 0.0.60", ver("vuer") == "0.0.60", ver("vuer"))
check("params_proto 2.13.2", ver("params_proto") == "2.13.2" or ver("params-proto") == "2.13.2",
      ver("params_proto") or ver("params-proto"))
cv_dists = sorted(d.metadata["Name"] for d in md.distributions()
                  if (d.metadata["Name"] or "").lower().startswith("opencv"))
cv_dirs = glob.glob(os.path.join(sys.prefix, "lib/python3.10/site-packages/cv2"))
import cv2
check("exactly one cv2", len(cv_dists) == 1 and len(cv_dirs) == 1,
      f"dists={cv_dists} dirs={len(cv_dirs)} cv2 {cv2.__version__}")
for m in ("unitree_sdk2py", "teleimager", "inspire_sdkpy", "televuer", "dex_retargeting",
          "casadi", "sshkeyboard", "torch", "nlopt", "pytransform3d", "logging_mp", "aiortc", "pymodbus"):
    try:
        mod = importlib.import_module(m)
        check(f"import {m}", True, f"{getattr(mod, '__version__', '')} {os.path.dirname(getattr(mod, '__file__', '') or '')}")
    except Exception as e:
        check(f"import {m}", False, repr(e)[:120])
print("ALL PASS" if ok_all else "SOME FAIL")
sys.stdout.flush()
os._exit(0 if ok_all else 1)
