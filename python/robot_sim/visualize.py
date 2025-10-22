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
    u_hist: Optional[np.ndarray] = None,
    R: float = 0.02,
    body_len: float = 0.2,
    follow: bool = True,
    window: float = 0.8,
    show: bool = True,
    save_path: Optional[str] = None,
):
    """Animate cart-wheel body with state and control plots.
    State order: [theta, dtheta, x, dx, phi, dphi]
    Control order: [T, Tp] (wheel torque, body torque)

    Args:
        x_hist: (N,6) state history
        dt: time step (s)
        u_hist: (N,2) control history (optional)
        R: wheel radius (m)
        body_len: body length drawn from wheel center (m)
        follow: if True, keep viewport centered around current x
        window: viewport width (m) when following
        show: if True, call plt.show()
        save_path: optional path to save a preview (png/mp4 if supported)
    """
    theta = x_hist[:, 0]
    x = x_hist[:, 2]
    t_arr = np.arange(len(theta)) * dt

    # Create figure with subplots: animation (top), states (middle), controls (bottom)
    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3, height_ratios=[2, 1, 1])
    
    # Animation subplot (spans both columns)
    ax_anim = fig.add_subplot(gs[0, :])
    ax_anim.set_aspect('equal', 'box')
    span = float(x.max() - x.min())
    default_width = max(window, 1.2 * span, 0.6)
    initial_xlim = (x.min() - default_width * 0.1, x.min() + default_width * 0.9)
    ax_anim.set_xlim(*initial_xlim)
    y_min = -0.05
    y_max = max(0.25, R + body_len + 0.05)
    ax_anim.set_ylim(y_min, y_max)
    ax_anim.grid(True, ls='--', alpha=0.3)
    ax_anim.set_title('Balance Animation', fontsize=12, fontweight='bold')

    ground, = ax_anim.plot([x.min()-1, x.max()+1], [0,0], 'k-', lw=1)
    wheel = plt.Circle((0, R), R, color='k', fill=False)
    ax_anim.add_patch(wheel)
    body_line, = ax_anim.plot([], [], 'r-', lw=2)
    
    # State plots
    ax_state1 = fig.add_subplot(gs[1, 0])
    ax_state2 = fig.add_subplot(gs[1, 1])
    
    # Plot full state history
    ax_state1.plot(t_arr, np.degrees(theta), 'b-', alpha=0.3, label='θ (body angle)')
    ax_state1.plot(t_arr, x * 100, 'g-', alpha=0.3, label='x (position × 100)')
    line_theta, = ax_state1.plot([], [], 'b-', lw=2)
    line_x, = ax_state1.plot([], [], 'g-', lw=2)
    ax_state1.axhline(0, color='k', ls='--', lw=0.5)
    ax_state1.set_xlabel('Time (s)')
    ax_state1.set_ylabel('θ (deg) / x (cm)')
    ax_state1.set_title('Body Angle & Position')
    ax_state1.legend(loc='upper right', fontsize=8)
    ax_state1.grid(True, alpha=0.3)
    
    ax_state2.plot(t_arr, np.degrees(x_hist[:, 1]), 'b-', alpha=0.3, label='dθ/dt')
    ax_state2.plot(t_arr, x_hist[:, 3] * 100, 'g-', alpha=0.3, label='dx/dt × 100')
    line_dtheta, = ax_state2.plot([], [], 'b-', lw=2)
    line_dx, = ax_state2.plot([], [], 'g-', lw=2)
    ax_state2.axhline(0, color='k', ls='--', lw=0.5)
    ax_state2.set_xlabel('Time (s)')
    ax_state2.set_ylabel('dθ/dt (deg/s) / dx/dt (cm/s)')
    ax_state2.set_title('Angular & Linear Velocity')
    ax_state2.legend(loc='upper right', fontsize=8)
    ax_state2.grid(True, alpha=0.3)
    
    # Control plots (if provided)
    if u_hist is not None:
        ax_ctrl1 = fig.add_subplot(gs[2, 0])
        ax_ctrl2 = fig.add_subplot(gs[2, 1])
        
        ax_ctrl1.plot(t_arr, u_hist[:, 0], 'r-', alpha=0.3, label='T (wheel)')
        line_T, = ax_ctrl1.plot([], [], 'r-', lw=2)
        ax_ctrl1.axhline(0, color='k', ls='--', lw=0.5)
        ax_ctrl1.set_xlabel('Time (s)')
        ax_ctrl1.set_ylabel('Torque (Nm)')
        ax_ctrl1.set_title('Wheel Torque (T)')
        ax_ctrl1.grid(True, alpha=0.3)
        
        ax_ctrl2.plot(t_arr, u_hist[:, 1], 'm-', alpha=0.3, label='Tp (body)')
        line_Tp, = ax_ctrl2.plot([], [], 'm-', lw=2)
        ax_ctrl2.axhline(0, color='k', ls='--', lw=0.5)
        ax_ctrl2.set_xlabel('Time (s)')
        ax_ctrl2.set_ylabel('Torque (Nm)')
        ax_ctrl2.set_title('Body Torque (Tp)')
        ax_ctrl2.grid(True, alpha=0.3)
    else:
        ax_ctrl1 = None
        ax_ctrl2 = None
        line_T = None
        line_Tp = None

    def update(i):
        xi = x[i]
        thetai = theta[i]
        ti = t_arr[i]
        
        # Update animation
        wheel.set_center((xi, R))
        base = np.array([xi, R])
        tip = base + body_len * np.array([np.sin(thetai), np.cos(thetai)])
        body_line.set_data([base[0], tip[0]], [base[1], tip[1]])
        
        if follow:
            half = default_width / 2.0
            new_xlim = (xi - half, xi + half)
            ax_anim.set_xlim(*new_xlim)
            ax_anim.set_ylim(y_min, y_max)
        ax_anim.set_title(f"t={ti:.2f}s | θ={np.degrees(thetai):.1f}° | x={xi*100:.1f}cm", 
                          fontsize=12, fontweight='bold')
        
        # Update state plots (highlight current time)
        line_theta.set_data(t_arr[:i+1], np.degrees(theta[:i+1]))
        line_x.set_data(t_arr[:i+1], x[:i+1] * 100)
        line_dtheta.set_data(t_arr[:i+1], np.degrees(x_hist[:i+1, 1]))
        line_dx.set_data(t_arr[:i+1], x_hist[:i+1, 3] * 100)
        
        artists = [wheel, body_line, line_theta, line_x, line_dtheta, line_dx]
        
        # Update control plots if available
        if u_hist is not None and line_T is not None:
            line_T.set_data(t_arr[:i+1], u_hist[:i+1, 0])
            line_Tp.set_data(t_arr[:i+1], u_hist[:i+1, 1])
            artists.extend([line_T, line_Tp])
        
        return artists

    n = len(theta)
    ani = animation.FuncAnimation(fig, update, frames=n, interval=dt*1000, blit=False)
    
    # Save if requested
    if save_path:
        try:
            update(n-1)
            fig.savefig(save_path, dpi=100, bbox_inches='tight')
        except Exception as e:
            print(f"Warning: failed to save preview to {save_path}: {e}")
    
    if show:
        plt.show()
    
    return ani
