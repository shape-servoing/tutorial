"""
Algorithm 1 of the tutorial in a few lines, without the interactive demos.

    python examples/algorithm1.py

Offline regression gives J0 at q0; the loop then drives the rod to the arch
target with the pseudoinverse law and the online Broyden update.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shape_servoing import (Plant, Q_ARCH, Q_NOMINAL, ShapeServoingLoop,  # noqa: E402
                            condition_number, offline_estimate, probe_motions, target_from_grasp)

ALPHA = 0.05

plant = Plant(q=Q_NOMINAL)                    # robot + rod + camera: command q, measure s
_, _, s_star = target_from_grasp(Q_ARCH)      # target recorded at another grasp: s* = s(q*)

# Offline estimation: C = 24 probes, J0 = dS dQ^+
J0, dS, residual = offline_estimate(plant.peek, Q_NOMINAL, probe_motions())
print(f"J0: residual {residual:.2f} ({residual / np.linalg.norm(dS):.0%} of ||dS||_F), "
      f"cond {condition_number(J0):.1f}")

# Algorithm 1: e_k, u_k = alpha J_k^+ e_k, move, measure, Broyden update
loop = ShapeServoingLoop(plant, s_star, J0, alpha=ALPHA)
e0 = np.linalg.norm(loop.error)
for k in range(1, 141):
    loop.step()
    if k % 20 == 0:
        print(f"k = {k:3d}   ||e_k|| / ||e_0|| = {np.linalg.norm(loop.error) / e0:.4f}"
              f"   nominal (1 - alpha)^k = {(1 - ALPHA) ** k:.4f}")
