#!/usr/bin/env python3
"""pack one run folder (same steps as scripts/pack.py: privacy / dash scan, SHA256SUMS inside, tar.gz + .sha256 beside it, extract and
verify, remove the extracted copy) without creating a pack run folder. pack(src_dir, target_tar_gz) -> (ok, sha256, bytes, message)"""
import hashlib
import os
import re
import shutil
import subprocess
import tempfile

_code = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pack.py")).read()
_ns = {"re": re, "hashlib": hashlib}
exec(_code[_code.index("pat = re.compile"):_code.index("hits = []")], _ns)


def _sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def pack(src, tgt):
    src = os.path.abspath(src.rstrip("/"))
    base = os.path.basename(src)
    if os.path.exists(tgt):
        return False, None, 0, "target exists"
    files = [os.path.join(dp, f) for dp, _, fs in os.walk(src) for f in fs]
    for fp in files:
        with open(fp, "r", errors="ignore") as fh:
            for line in fh:
                if _ns["pat"].search(line) or _ns["serial_hit"](line):
                    return False, None, 0, "scan hit in %s" % os.path.relpath(fp, src)
    with open(os.path.join(src, "SHA256SUMS"), "w") as fh:
        for f in sorted(os.path.relpath(x, src) for x in files if os.path.relpath(x, src) != "SHA256SUMS"):
            fh.write("%s  %s\n" % (_sha(os.path.join(src, f)), f))
    subprocess.run(["tar", "-czf", tgt, "-C", os.path.dirname(src), base], check=True)
    h = _sha(tgt)
    open(tgt + ".sha256", "w").write("%s  %s\n" % (h, os.path.basename(tgt)))
    d = tempfile.mkdtemp(dir="/tmp")
    try:
        subprocess.run(["tar", "-xzf", tgt, "-C", d], check=True)
        ok = subprocess.run(["sha256sum", "-c", "SHA256SUMS", "--quiet"], cwd=os.path.join(d, base)).returncode == 0
    finally:
        shutil.rmtree(d)
    return ok, h, os.path.getsize(tgt), "verified" if ok else "VERIFY FAILED"
