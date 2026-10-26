#!/usr/bin/env python3
"""Summarise a tegrastats log: mean/max of all-core CPU %, GR3D %, RAM, CPU/GPU/tj temps."""
import re, sys
for path in sys.argv[1:]:
    cpu, gpu, ram, tcpu, tgpu, tj = [], [], [], [], [], []
    for line in open(path):
        m = re.search(r"CPU \[([^\]]*)\]", line)
        if m:
            vals = [int(x.split("%")[0]) for x in m.group(1).split(",") if "%" in x]
            cpu.append(sum(vals) / len(vals))
        m = re.search(r"GR3D_FREQ (\d+)%", line); gpu += [int(m.group(1))] if m else []
        m = re.search(r"RAM (\d+)/", line); ram += [int(m.group(1))] if m else []
        m = re.search(r"CPU@([\d.]+)C", line); tcpu += [float(m.group(1))] if m else []
        m = re.search(r"GPU@([\d.]+)C", line); tgpu += [float(m.group(1))] if m else []
        m = re.search(r"tj@([\d.]+)C", line); tj += [float(m.group(1))] if m else []
    f = lambda x: f"{sum(x)/len(x):.1f}/{max(x):.1f}" if x else "n/a"
    print(f"{path.split('/')[-1]}: n={len(cpu)} s | CPU all-core % mean/max {f(cpu)} | GR3D % {f(gpu)} | "
          f"RAM MB {f(ram)} | CPU C {f(tcpu)} | GPU C {f(tgpu)} | tj C {f(tj)}")
