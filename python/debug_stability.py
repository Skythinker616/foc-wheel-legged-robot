#!/usr/bin/env python3
"""Debug script to check system stability and eigenvalues."""
import numpy as np
from robot_sim.system import build_linear_model, discretize, compute_dlqr
from robot_sim.lqr import lqr_k

L0 = 0.07
DT = 0.005

print("="*60)
print(f"Checking linear model stability for L0={L0}, dt={DT}")
print("="*60)

# Build continuous model
model = build_linear_model(L0)
A = model.A
B = model.B

print("\n[Continuous A matrix]")
print(A)
print("\n[Continuous B matrix]")
print(B)

# Check continuous eigenvalues
eig_c = np.linalg.eigvals(A)
print("\n[Continuous eigenvalues]")
for i, e in enumerate(eig_c):
    print(f"  λ_{i+1} = {e:.6f}")
print(f"Max real part: {np.max(np.real(eig_c)):.6f}")
if np.max(np.real(eig_c)) > 0:
    print("  ⚠️  UNSTABLE continuous system (positive real eigenvalue)")
else:
    print("  ✓ Continuous system is stable")

# Discretize
Ad, Bd = discretize(A, B, DT)
print(f"\n[Discrete Ad matrix (dt={DT})]")
print(Ad)
print("\n[Discrete Bd matrix]")
print(Bd)

# Check discrete eigenvalues
eig_d = np.linalg.eigvals(Ad)
print("\n[Discrete eigenvalues]")
for i, e in enumerate(eig_d):
    print(f"  λ_{i+1} = {e:.6f}, |λ|={np.abs(e):.6f}")
max_mag = np.max(np.abs(eig_d))
print(f"Max magnitude: {max_mag:.6f}")
if max_mag >= 1.0:
    print("  ⚠️  UNSTABLE open-loop discrete system (eigenvalue magnitude >= 1)")
else:
    print("  ✓ Open-loop discrete system is stable")

# MATLAB fitted LQR gain
K_matlab = lqr_k(L0)
print(f"\n[MATLAB-fitted LQR gain K for L0={L0}]")
print(K_matlab)

Acl_matlab = Ad - Bd @ K_matlab
eig_cl_matlab = np.linalg.eigvals(Acl_matlab)
print("\n[Closed-loop eigenvalues with MATLAB K]")
for i, e in enumerate(eig_cl_matlab):
    print(f"  λ_{i+1} = {e:.6f}, |λ|={np.abs(e):.6f}")
max_mag_cl = np.max(np.abs(eig_cl_matlab))
print(f"Max magnitude: {max_mag_cl:.6f}")
if max_mag_cl >= 1.0:
    print("  ⚠️  UNSTABLE closed-loop with MATLAB K (eigenvalue magnitude >= 1)")
    print("  → Simulation will diverge!")
else:
    print("  ✓ Closed-loop with MATLAB K is stable")

# Recompute LQR with Python dlqr using same Q/R as MATLAB
print("\n" + "="*60)
print("Recomputing LQR with Python scipy.linalg...")
print("="*60)
Q = np.diag([1, 10, 100, 20, 1000, 1])
R = np.diag([1, 1])
print(f"Q = diag{list(np.diag(Q))}")
print(f"R = diag{list(np.diag(R))}")

K_python = compute_dlqr(Ad, Bd, Q, R)
print(f"\n[Python-computed DLQR gain K]")
print(K_python)

Acl_python = Ad - Bd @ K_python
eig_cl_python = np.linalg.eigvals(Acl_python)
print("\n[Closed-loop eigenvalues with Python K]")
for i, e in enumerate(eig_cl_python):
    print(f"  λ_{i+1} = {e:.6f}, |λ|={np.abs(e):.6f}")
max_mag_cl_py = np.max(np.abs(eig_cl_python))
print(f"Max magnitude: {max_mag_cl_py:.6f}")
if max_mag_cl_py >= 1.0:
    print("  ⚠️  UNSTABLE closed-loop with Python K (eigenvalue magnitude >= 1)")
    print("  → Need to adjust Q/R or dt")
else:
    print("  ✓ Closed-loop with Python K is STABLE")

print("\n" + "="*60)
print("Gain comparison:")
print(f"K difference (Frobenius norm): {np.linalg.norm(K_matlab - K_python):.6f}")
print("="*60)
