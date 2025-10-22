import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from typing import Tuple, Callable, Optional
from .leg import leg_pos, _phi2_from_phi1_phi4, L1, L2, L3, L4, L5


def draw_leg(ax, phi1: float, phi4: float):
    """Draw the four-bar leg and virtual leg on given axes."""
    # Points: A(0,0) -> B, C, D -> E(L5,0)
    A = np.array([0.0, 0.0])
    E = np.array([L5, 0.0])
    B = np.array([L1*np.cos(phi1), L1*np.sin(phi1)])
    D = np.array([L5 + L4*np.cos(phi4), L4*np.sin(phi4)])
    phi2 = _phi2_from_phi1_phi4(phi1, phi4)
    C = B + np.array([L2*np.cos(phi2), L2*np.sin(phi2)])

    # Virtual leg
    pose = leg_pos(phi1, phi4)
    V_base = np.array([L5/2, 0.0])
    V_tip = V_base + pose.l0 * np.array([np.cos(pose.phi0), np.sin(pose.phi0)])

    # Clear and draw
    ax.clear()
    ax.set_aspect('equal', 'box')
    ax.set_xlim(-0.05, L5+0.05)
    ax.set_ylim(-0.05, 0.15)
    ax.grid(True, ls='--', alpha=0.3)

    # Linkages
    ax.plot([A[0], B[0]], [A[1], B[1]], 'k-', lw=2)
    ax.plot([B[0], C[0]], [B[1], C[1]], 'b-', lw=2)
    ax.plot([D[0], C[0]], [D[1], C[1]], 'b-', lw=2)
    ax.plot([E[0], D[0]], [E[1], D[1]], 'k-', lw=2)

    # Virtual leg
    ax.plot([V_base[0], V_tip[0]], [V_base[1], V_tip[1]], 'r-', lw=2)
    ax.scatter([A[0], B[0], C[0], D[0], E[0], V_base[0]], [A[1], B[1], C[1], D[1], E[1], V_base[1]], c='k', s=15)
    ax.set_title(f"l0={pose.l0:.3f} m, phi0={np.degrees(pose.phi0):.1f} deg")


def animate_leg(phi1_func: Callable[[float], float],
                 phi4_func: Callable[[float], float],
                 duration: float = 5.0, fps: int = 30):
    fig, ax = plt.subplots(figsize=(6,4))
    n_frames = int(duration*fps)

    def update(i):
        t = i / fps
        draw_leg(ax, phi1_func(t), phi4_func(t))
        return []

    ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=1000/fps, blit=False)
    plt.show()
    return ani


def animate_balance(
    x_hist: np.ndarray,
    dt: float,
    R: float = 0.02,
    body_len: float = 0.2,
    follow: bool = True,
    window: float = 0.8,
    show: bool = True,
    save_path: Optional[str] = None,
    aspect_equal: bool = False,
):
    """Animate simple cart-wheel body using theta and x from x_hist.
    State order: [theta, dtheta, x, dx, phi, dphi]

    Args:
        x_hist: (N,6) state history
        dt: time step (s)
        R: wheel radius (m)
        body_len: body length drawn from wheel center (m)
        follow: if True, keep viewport centered around current x
        window: viewport width (m) when following
        show: if True, call plt.show()
        save_path: optional path to save a preview (png/mp4 if supported)
    """
    theta = x_hist[:, 0]
    x = x_hist[:, 2]

    fig, ax = plt.subplots(figsize=(7, 4))
    if aspect_equal:
        ax.set_aspect('equal', 'box')
    else:
        ax.set_aspect('auto')
    span = float(x.max() - x.min())
    default_width = max(window, 1.2 * span, 0.6)
    initial_xlim = (x.min() - default_width * 0.1, x.min() + default_width * 0.9)
    ax.set_xlim(*initial_xlim)
    y_min = -0.05
    y_max = max(0.25, R + body_len + 0.05)
    ax.set_ylim(y_min, y_max)
    ax.grid(True, ls='--', alpha=0.3)

    ground, = ax.plot([x.min()-1, x.max()+1], [0,0], 'k-', lw=1)
    wheel = plt.Circle((0, R), R, color='k', fill=False)
    ax.add_patch(wheel)
    body_line, = ax.plot([], [], 'r-', lw=2)

    def update(i):
        xi = x[i]
        thetai = theta[i]
        # Update wheel
        wheel.set_center((xi, R))
        # Body line from wheel center upward at angle theta
        base = np.array([xi, R])
        tip = base + body_len * np.array([np.sin(thetai), np.cos(thetai)])
        body_line.set_data([base[0], tip[0]], [base[1], tip[1]])
        if follow:
            half = default_width / 2.0
            new_xlim = (xi - half, xi + half)
            ax.set_xlim(*new_xlim)
            # keep vertical range stable for visibility
            ax.set_ylim(y_min, y_max)
        ax.set_title(f"t={i*dt:.2f}s, theta={np.degrees(thetai):.1f} deg, x={xi:.2f} m")
        return [wheel, body_line]

    n = len(theta)
    ani = animation.FuncAnimation(fig, update, frames=n, interval=dt*1000, blit=False)
    # Save if requested
    if save_path:
        try:
            # Prefer saving a snapshot if ffmpeg isn't available
            update(n-1)
            fig.savefig(save_path, dpi=100, bbox_inches='tight')
        except Exception as e:
            print(f"Warning: failed to save preview to {save_path}: {e}")
    if show:
        plt.show()
    return ani
