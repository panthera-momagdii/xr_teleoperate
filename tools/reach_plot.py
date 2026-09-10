#!/usr/bin/env python3
"""Render the reach map and work out what XR_WRIST_Z_OFFSET should be.

    python tools/reach_plot.py --npz logs/overnight/reach_map.npz --out-dir logs/overnight

Offline, read-only. Produces three xy slices (one per height) plus a side view, and
prints the offset recommendation with the arithmetic shown.
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# Head -> IK base, mirrored from tv_wrapper.py:117-118 (see teleop/robot_control/wrist_offset.py)
HEAD_TO_WAIST_Z = 0.45

# What an adult operator's hand height looks like RELATIVE TO THE HEADSET. Measured
# from nothing -- these are anthropometric estimates, and they are the one soft input
# in this analysis, so they are stated here rather than buried.
OPERATOR_HAND_Z = {
    "hands at belly, elbows relaxed": -0.50,
    "hands at chest / ready pose":    -0.35,
    "hands at shoulder height":       -0.25,
    "hands at head height":            0.00,
}


def slice_plot(ax, ys, zs, ok_yz, title):
    cmap = ListedColormap(["#f4f1ea", "#2f6f4f"])
    ax.pcolormesh(ys, zs, ok_yz.T.astype(float), cmap=cmap, vmin=0, vmax=1,
                  shading="nearest", edgecolors="none")
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("y (m, +left)", fontsize=8)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    ax.axhline(0.0, color="#999", lw=0.6, ls=":")
    ax.axvline(0.0, color="#999", lw=0.6, ls=":")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default="logs/overnight/reach_map.npz")
    ap.add_argument("--out-dir", default="logs/overnight")
    ap.add_argument("--x-slice", type=float, default=0.30)
    args = ap.parse_args()

    d = np.load(args.npz, allow_pickle=True)
    xs, ys, zs = d["x"], d["y"], d["z"]
    os.makedirs(args.out_dir, exist_ok=True)
    written = []

    # ---- three xy slices, at three heights ------------------------------------
    for z_target in (-0.10, 0.20, 0.45):
        k = int(np.argmin(np.abs(zs - z_target)))
        fig, axes = plt.subplots(1, 2, figsize=(8.2, 3.6))
        for ax, side in zip(axes, ("left", "right")):
            ok = d[f"{side}_ok_full"][:, :, k]          # (x, y)
            cmap = ListedColormap(["#f4f1ea", "#2f6f4f"])
            ax.pcolormesh(ys, xs, ok.astype(float), cmap=cmap, vmin=0, vmax=1,
                          shading="nearest")
            ax.set_title(f"{side} arm", fontsize=9)
            ax.set_xlabel("y (m, +left)", fontsize=8)
            ax.set_ylabel("x (m, forward)", fontsize=8)
            ax.set_aspect("equal")
            ax.tick_params(labelsize=7)
            ax.plot(0.1002 if side == "left" else -0.1002, 0.0, marker="o",
                    ms=5, mfc="#c0392b", mec="white", mew=0.8, zorder=5)
        fig.suptitle(f"G1_29 reachable wrist positions, wrist pointing forward   "
                     f"z = {zs[k]:+.2f} m  (red dot = shoulder)", fontsize=10)
        fig.tight_layout()
        path = os.path.join(args.out_dir, f"reach_map_xy_z{zs[k]:+.2f}.png".replace("+", "p").replace("-", "m"))
        fig.savefig(path, dpi=150)
        plt.close(fig)
        written.append(path)

    # ---- a side view at the x slice, which is where the story is ---------------
    i = int(np.argmin(np.abs(xs - args.x_slice)))
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 4.0))
    for ax, side in zip(axes, ("left", "right")):
        slice_plot(ax, ys, zs, d[f"{side}_ok_full"][i], f"{side} arm")
        ax.set_ylabel("z (m, +up)", fontsize=8)
        ax.axhline(0.2918, color="#c0392b", lw=1.0, ls="--")
        ax.text(ys[0], 0.2918, " shoulder", color="#c0392b", fontsize=7, va="bottom")
        for label, hz in OPERATOR_HAND_Z.items():
            ax.axhline(hz + HEAD_TO_WAIST_Z, color="#2c6fbb", lw=0.8, ls="-", alpha=0.7)
            ax.text(ys[-1], hz + HEAD_TO_WAIST_Z, label.split(",")[0] + " ",
                    color="#2c6fbb", fontsize=6, va="bottom", ha="right")
    fig.suptitle(f"Where the operator's hands land vs what the arm can reach  "
                 f"(x = {xs[i]:+.2f} m, no offset)", fontsize=10)
    fig.tight_layout()
    path = os.path.join(args.out_dir, "reach_map_side_x0p30.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # ---- the numbers -----------------------------------------------------------
    print(f"x slice used: {xs[i]:+.2f} m\n")
    bands = {}
    for side in ("left", "right"):
        ok = d[f"{side}_ok_full"][i]
        idx = np.flatnonzero(ok.any(axis=0))
        band = (float(zs[idx[0]]), float(zs[idx[-1]]))
        bands[side] = band
        print(f"{side} arm reachable z at x={xs[i]:+.2f}: "
              f"{band[0]:+.2f} .. {band[1]:+.2f} m   (span {band[1]-band[0]:.2f} m, "
              f"centre {(band[0]+band[1])/2:+.2f})")

    lo = max(bands["left"][0], bands["right"][0])
    hi = min(bands["left"][1], bands["right"][1])
    centre = (lo + hi) / 2.0
    print(f"\nboth arms: {lo:+.2f} .. {hi:+.2f} m, centre {centre:+.2f} m")

    print("\nwhere the operator's hands land today (XR_WRIST_Z_OFFSET=0):")
    for label, hz in OPERATOR_HAND_Z.items():
        target = hz + HEAD_TO_WAIST_Z
        mark = "inside" if lo <= target <= hi else ">>> OUTSIDE <<<"
        print(f"    {label:<34} head-relative {hz:+.2f} -> target z {target:+.2f}   {mark}")

    ready = OPERATOR_HAND_Z["hands at chest / ready pose"] + HEAD_TO_WAIST_Z
    rec = centre - ready
    print(f"\nrecommendation")
    print(f"    to put the 'hands at chest / ready pose' posture ({ready:+.2f}) at the")
    print(f"    centre of the reachable band ({centre:+.2f}):")
    print(f"        XR_WRIST_Z_OFFSET = {centre:+.2f} - ({ready:+.2f}) = {rec:+.2f}")
    belly = OPERATOR_HAND_Z["hands at belly, elbows relaxed"] + HEAD_TO_WAIST_Z
    print(f"    with that offset, 'hands at belly' ({belly:+.2f}) maps to "
          f"{belly + rec:+.2f}, which is {'inside' if lo <= belly + rec <= hi else 'OUTSIDE'} the band.")
    print(f"\n    START WITH  XR_WRIST_Z_OFFSET={rec:.2f}  and adjust by feel.")

    print("\nfigures written:")
    for p in written:
        print(f"    {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
