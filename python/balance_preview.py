# Generate a non-GUI preview image for the balance animation
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # non-interactive backend

from robot_sim.system import build_linear_model, discretize, compute_dlqr, simulate_closed_loop
from robot_sim.visualize import animate_balance

DT = 0.005
SIM_TIME = 4.0

if __name__ == "__main__":
    L0 = 0.07
    model = build_linear_model(L0)
    Ad, Bd = discretize(model.A, model.B, DT)
    
    # Compute stable LQR gain
    Q = np.diag([1, 10, 100, 20, 1000, 1])
    R = np.diag([1, 1])
    K = compute_dlqr(Ad, Bd, Q, R)

    x0 = np.array([
        np.deg2rad(5.0),  # theta
        0.0,              # dtheta
        0.0,              # x
        0.0,              # dx
        0.0,              # phi
        0.0,              # dphi
    ])

    steps = int(SIM_TIME/DT)
    x_hist, u_hist = simulate_closed_loop(Ad, Bd, K, x0, steps)

    out_path = os.path.join(os.path.dirname(__file__), 'balance_preview.png')
    animate_balance(x_hist, DT, u_hist=u_hist, show=False, save_path=out_path, follow=True)
    print(f"Saved preview to {out_path}")
