#!/usr/bin/env python3
"""Resolve and describe the TLS certificate televuer will actually serve.

Read-only. No DDS, no network, no robot. Safe to import from the launcher and safe to
run at any time.

Why this exists
---------------
On 2026-09-09 a regenerated `cert.pem` silently invalidated the Quest's stored
certificate exception. Nothing printed a fingerprint, so the first symptom was a
headset that would not connect, ten minutes into a robot session. The launcher now
prints one line at startup naming the resolved path, its sha256 and its SAN list, so a
changed certificate is visible before anything is powered up rather than after.

The resolution here MIRRORS `televuer.TeleVuer.__init__`
(teleop/televuer/src/televuer/televuer.py:69-89) and must be kept in step with it:

  1. $XR_TELEOP_CERT and $XR_TELEOP_KEY -- only if BOTH are set
  2. ~/.config/xr_teleoperate/{cert,key}.pem -- only if BOTH exist
  3. the televuer package root: teleop/televuer/{cert,key}.pem

Step 2 beating step 3 is the part that surprises people: editing the copy in the repo
does nothing while a copy exists under ~/.config.

Usage:
    python tools/cert_info.py            # human-readable block
    python tools/cert_info.py --oneline  # the single line the launcher prints
"""

import argparse
import hashlib
import os
import subprocess
import sys
from pathlib import Path

# The televuer package root, as televuer.py computes it:
#   Path(__file__).resolve().parent.parent.parent  from src/televuer/televuer.py
#   = teleop/televuer/
REPO_ROOT = Path(__file__).resolve().parent.parent
TELEVUER_ROOT = REPO_ROOT / "teleop" / "televuer"
USER_CONF_DIR = Path.home() / ".config" / "xr_teleoperate"


def resolve_cert_key(cert_file=None, key_file=None):
    """Return (cert_path, key_path, how) exactly as televuer would choose them.

    `how` is a short tag naming which of the three rules won, for the log line.
    Either path may be a file that does not exist -- rule 3 is an unconditional
    fallback in televuer too, and reporting a missing file is more useful than
    pretending there is nothing to report.
    """
    if cert_file is not None and key_file is not None:
        return Path(cert_file), Path(key_file), "explicit argument"

    env_cert = os.getenv("XR_TELEOP_CERT")
    env_key = os.getenv("XR_TELEOP_KEY")
    if env_cert and env_key:
        return (Path(cert_file or env_cert), Path(key_file or env_key),
                "$XR_TELEOP_CERT/$XR_TELEOP_KEY")

    user_cert = USER_CONF_DIR / "cert.pem"
    user_key = USER_CONF_DIR / "key.pem"
    if user_cert.exists() and user_key.exists():
        return (Path(cert_file or user_cert), Path(key_file or user_key),
                "~/.config/xr_teleoperate")

    return (Path(cert_file or TELEVUER_ROOT / "cert.pem"),
            Path(key_file or TELEVUER_ROOT / "key.pem"),
            "televuer package root")


def sha256_of(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def _openssl(path, *args, timeout=5):
    """Run `openssl x509 -in path -noout <args>`; return stdout or None.

    Never raises: this is a diagnostic line, and a box without openssl must still be
    able to start a teleop session.
    """
    try:
        out = subprocess.run(
            ["openssl", "x509", "-in", str(path), "-noout", *args],
            capture_output=True, text=True, timeout=timeout, check=False)
        return out.stdout.strip() if out.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def san_list(path):
    """Return the SAN entries as a list of short strings, or [] if unavailable.

    'DNS:localhost, IP Address:192.168.8.7' -> ['localhost', '192.168.8.7']
    """
    raw = _openssl(path, "-ext", "subjectAltName")
    if not raw:
        return []
    names = []
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith("X509v3") or not line:
            continue
        for part in line.split(","):
            part = part.strip()
            for prefix in ("DNS:", "IP Address:", "URI:", "email:"):
                if part.startswith(prefix):
                    names.append(part[len(prefix):])
                    break
    return names


def cert_summary(cert_file=None, key_file=None):
    """Everything the launcher and the runbook want to know, as a dict."""
    cert, key, how = resolve_cert_key(cert_file, key_file)
    digest = sha256_of(cert)
    return {
        "cert": str(cert),
        "key": str(key),
        "how": how,
        "exists": Path(cert).exists(),
        "key_exists": Path(key).exists(),
        "sha256": digest,
        "sans": san_list(cert) if digest else [],
        "subject": _openssl(cert, "-subject") if digest else None,
        "not_after": _openssl(cert, "-enddate") if digest else None,
    }


def oneline(summary=None):
    """The single startup line. Deliberately one line: it is read at a glance."""
    s = summary or cert_summary()
    if not s["exists"]:
        return (f"[cert] MISSING {s['cert']} (via {s['how']}) -- "
                f"televuer will fail to start TLS")
    sans = ", ".join(s["sans"]) if s["sans"] else "<could not read SANs>"
    return (f"[cert] {s['cert']} (via {s['how']}) sha256={s['sha256'][:16]}... "
            f"SAN=[{sans}]")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--oneline", action="store_true",
                    help="print only the launcher's startup line")
    ap.add_argument("--cert", default=None, help="override the cert path")
    ap.add_argument("--key", default=None, help="override the key path")
    args = ap.parse_args()

    s = cert_summary(args.cert, args.key)
    if args.oneline:
        print(oneline(s))
        return 0 if s["exists"] else 1

    print(f"resolved by : {s['how']}")
    print(f"cert        : {s['cert']}{'' if s['exists'] else '   *** MISSING ***'}")
    print(f"key         : {s['key']}{'' if s['key_exists'] else '   *** MISSING ***'}")
    if not s["exists"]:
        print("\nThe three rules, in order (televuer.py:69-89):")
        print(f"  1. $XR_TELEOP_CERT + $XR_TELEOP_KEY  "
              f"({os.getenv('XR_TELEOP_CERT') or 'unset'})")
        print(f"  2. {USER_CONF_DIR}/cert.pem + key.pem")
        print(f"  3. {TELEVUER_ROOT}/cert.pem + key.pem")
        return 1
    print(f"sha256      : {s['sha256']}")
    print(f"subject     : {s['subject'] or '<openssl unavailable>'}")
    print(f"expires     : {s['not_after'] or '<openssl unavailable>'}")
    print(f"SAN         : {', '.join(s['sans']) if s['sans'] else '<none readable>'}")
    print()
    print("If the Quest refuses to connect after this fingerprint changed, its stored")
    print("certificate exception is for the OLD cert. Clear the site data for")
    print("https://<ip>:8012/ on the headset and accept the warning once more.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
