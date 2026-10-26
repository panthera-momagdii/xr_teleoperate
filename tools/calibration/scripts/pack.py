#!/usr/bin/env python3
"""pack: packs a session folder into /home/unitree/g1rs_<name>.tar.gz plus .sha256: writes SHA256SUMS inside first, scans for LAN /
Tailscale addresses, hand / camera serials and em or en dashes, excludes BYTE_ORDER_RAW files, verifies an extracted copy. Dry-run
lists what would be packed.
   python scripts/pack.py <session_dir> [--name NAME] [--live]"""
import hashlib
import os
import re
import shutil
import subprocess
import tempfile
import cw

p = cw.parser(__doc__)
p.add_argument("session_dir")
p.add_argument("--name", default=None)
a = p.parse_args()
src = os.path.abspath(a.session_dir.rstrip("/"))
base = os.path.basename(src)
name = a.name or base
tgt = "/home/unitree/%s.tar.gz" % (name if name.startswith("g1rs_") else "g1rs_" + name)
run = cw.Run("pack", a)
files = [os.path.join(dp, f) for dp, _, fs in os.walk(src) for f in fs if not f.startswith("BYTE_ORDER_RAW")]
size = sum(os.path.getsize(f) for f in files)
pat = re.compile(r"(192\.168\.\d+\.\d+|100\.(6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.\d+\.\d+|\u2014|\u2013)")
# hand and camera serials are not kept in any file (house rule): candidates of their shape are hashed and compared
SERIAL_SHA256 = {"0aace538549119d2da07f5eb7f791847fd66e630f768b8b10c71224f20114620",
                 "6230a5249a311122284abcad61282aab69ed0fc2f960d2142163a48d1d72d65b",
                 "94abf6f874750635029fa16e0b34c3b9bdd1a6bd2bbd15cfd80c734c0a9f1a97"}
cand = re.compile(r"(?=(\d{3}[A-Z]{2}\d[A-Z]))|(?<!\d)(\d{12})(?!\d)")


def serial_hit(line):
    return any(hashlib.sha256(x.encode()).hexdigest() in SERIAL_SHA256 for m in cand.finditer(line) for x in m.groups() if x)
hits = []
for fpath in files:
    try:
        with open(fpath, "r", errors="ignore") as fh:
            for i, line in enumerate(fh):
                if pat.search(line) or serial_hit(line):
                    hits.append("%s:%d" % (os.path.relpath(fpath, src), i + 1))
                    break
    except OSError:
        pass
print("source %s: %d files, %.1f MB; target %s" % (src, len(files), size / 1e6, tgt))
print("privacy / dash scan: %s" % (hits[:10] or "clean"))
if not a.live:
    run.finish("DRY_RUN")
else:
    if hits:
        raise SystemExit("STOP: scan hits, nothing packed")
    if os.path.exists(tgt):
        raise SystemExit("STOP: %s exists" % tgt)
    with open(os.path.join(src, "SHA256SUMS"), "w") as fh:
        for f in sorted(os.path.relpath(x, src) for x in files if not x.endswith("/SHA256SUMS") and os.path.relpath(x, src) != "SHA256SUMS"):
            fh.write("%s  %s\n" % (cw.sha(os.path.join(src, f)), f))
    subprocess.run(["tar", "--exclude=BYTE_ORDER_RAW*", "-czf", tgt, "-C", os.path.dirname(src), base], check=True)
    h = cw.sha(tgt)
    open(tgt + ".sha256", "w").write("%s  %s\n" % (h, os.path.basename(tgt)))
    n = int(subprocess.run("tar -tzf %s | grep -v '/$' | wc -l" % tgt, shell=True, capture_output=True, text=True).stdout)
    d = tempfile.mkdtemp(dir="/tmp")
    subprocess.run(["tar", "-xzf", tgt, "-C", d], check=True)
    ok = subprocess.run(["sha256sum", "-c", "SHA256SUMS", "--quiet"], cwd=os.path.join(d, base)).returncode == 0
    shutil.rmtree(d)                                   # the extracted copy was only for the check
    print("packed %s  %d bytes  %d files  sha256 %s  extracted copy verifies: %s" % (tgt, os.path.getsize(tgt), n, h, ok))
    run.finish("DONE" if ok else "VERIFY_FAILED")
