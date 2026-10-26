#!/usr/bin/env python3
"""arm_* scripts: DRY-RUN ONLY. They print what they would publish and publish nothing; --live is always refused (LOCKS arm_live)."""
import cw

LEFT = list(range(15, 22))
RIGHT = list(range(22, 29))


def refuse(a):
    if a.live:
        cw.refuse_if_locked("arm_live", True)
        raise SystemExit("arm scripts never go live in this workspace")


def args(p):
    p.add_argument("--side", default="both", choices=["left", "right", "both"])
    p.add_argument("--kp", type=float, default=40.0)
    p.add_argument("--kd", type=float, default=1.0)


def joints(side):
    return {"left": LEFT, "right": RIGHT, "both": LEFT + RIGHT}[side]
