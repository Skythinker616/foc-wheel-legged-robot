import numpy as np
from dataclasses import dataclass

# Geometry constants from matlab/leg_func_calc.m
L1 = 0.05  # l1
L2 = 0.105 # l2
L3 = L2    # l3
L4 = L1    # l4
L5 = 0.06  # l5 (hip spacing)

@dataclass
class LegPose:
    l0: float  # virtual leg length (m)
    phi0: float  # virtual leg angle (rad)

@dataclass
class LegVelocity:
    dl0: float
    dphi0: float


def _phi2_from_phi1_phi4(phi1: float, phi4: float) -> float:
    """Solve internal angle phi2 using geometry (four-bar linkage closure).
    This reproduces the MATLAB code in leg_func_calc.m.
    """
    xb = L1 * np.cos(phi1)
    yb = L1 * np.sin(phi1)
    xd = L5 + L4 * np.cos(phi4)
    yd = L4 * np.sin(phi4)

    A0 = 2 * L2 * (xd - xb)
    B0 = 2 * L2 * (yd - yb)
    C0 = L2**2 + (xd - xb)**2 + (yd - yb)**2 - L3**2

    # Choose the elbow-down solution consistent with MATLAB 2*atan(...) branch
    inside = A0**2 + B0**2 - C0**2
    inside = max(inside, 0.0)
    phi2 = 2 * np.arctan2(B0 + np.sqrt(inside), A0 + C0)
    return phi2


def leg_pos(phi1: float, phi4: float) -> LegPose:
    """Compute virtual leg pose [l0, phi0] from joint angles phi1, phi4 (rad)."""
    phi2 = _phi2_from_phi1_phi4(phi1, phi4)

    xb = L1 * np.cos(phi1)
    yb = L1 * np.sin(phi1)
    xc = xb + L2 * np.cos(phi2)
    yc = yb + L2 * np.sin(phi2)

    l0 = np.hypot(xc - L5 / 2, yc)
    phi0 = np.arctan2(yc, xc - L5 / 2)
    return LegPose(l0=float(l0), phi0=float(phi0))


def leg_jacobian(phi1: float, phi4: float) -> np.ndarray:
    """Jacobian J s.t. [dl0; dphi0] = J @ [dphi1; dphi4].
    We compute it by finite differences for robustness and simplicity.
    """
    eps = 1e-7
    base = leg_pos(phi1, phi4)

    # partial w.r.t phi1
    p1 = leg_pos(phi1 + eps, phi4)
    J_col1 = np.array([(p1.l0 - base.l0) / eps, (p1.phi0 - base.phi0) / eps])

    # partial w.r.t phi4
    p4 = leg_pos(phi1, phi4 + eps)
    J_col2 = np.array([(p4.l0 - base.l0) / eps, (p4.phi0 - base.phi0) / eps])

    J = np.column_stack([J_col1, J_col2])
    return J


def leg_spd(dphi1: float, dphi4: float, phi1: float, phi4: float) -> LegVelocity:
    """Compute virtual leg speed [dl0, dphi0] from joint speeds and angles."""
    J = leg_jacobian(phi1, phi4)
    v = J @ np.array([dphi1, dphi4], dtype=float)
    return LegVelocity(dl0=float(v[0]), dphi0=float(v[1]))


def leg_conv(F: float, Tp: float, phi1: float, phi4: float) -> np.ndarray:
    """VMC mapping: [T1, T2] = J^T @ [F, Tp].
    F: axial thrust along virtual leg (N)
    Tp: torque about hip (Nm)
    Returns motor torques for joints (phi1, phi4).
    """
    J = leg_jacobian(phi1, phi4)
    tau = J.T @ np.array([F, Tp], dtype=float)
    return tau
