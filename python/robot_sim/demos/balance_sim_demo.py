#!/usr/bin/env python3
import numpy as np
from robot_sim.system import build_linear_model, discretize, compute_dlqr, simulate_closed_loop
from robot_sim.visualize import animate_balance

DT = 0.005
SIM_TIME = 6.0

if __name__ == "__main__":
    L0 = 0.07  # nominal leg length
    # Build and discretize linear model
    model = build_linear_model(L0)
    Ad, Bd = discretize(model.A, model.B, DT)

    # Compute LQR gain using Python (same Q/R as MATLAB)
    Q = np.diag([1, 10, 100, 20, 1000, 1])
    R = np.diag([1, 1])
    K = compute_dlqr(Ad, Bd, Q, R)
    
    print(f"LQR gain K computed for L0={L0}:")
    print(K)
    
    # Verify stability
    Acl = Ad - Bd @ K
    eigs = np.linalg.eigvals(Acl)
    max_eig = np.max(np.abs(eigs))
    print(f"Closed-loop max eigenvalue magnitude: {max_eig:.6f}")
    if max_eig < 1.0:
        print("✓ System is stable")
    else:
        print("⚠️  System is UNSTABLE - simulation may diverge")

    # Initial state: small tilt and position offset
    x0 = np.array([
        np.deg2rad(5.0),  # theta
        0.0,              # dtheta
        0.0,              # x
        0.0,              # dx
        0.0,              # phi
        0.0,              # dphi
    ])

    steps = int(SIM_TIME/DT)
    x_hist = simulate_closed_loop(Ad, Bd, K, x0, steps)
    
    print(f"\nSimulation complete: {steps+1} steps")
    print(f"Final state: theta={np.degrees(x_hist[-1,0]):.2f} deg, x={x_hist[-1,2]:.3f} m")

    # Animate with robust defaults: follow the cart, auto aspect (prevents y squeeze),
    # and a reasonable horizontal window.
    animate_balance(x_hist, DT, follow=True, window=0.8, aspect_equal=False)
