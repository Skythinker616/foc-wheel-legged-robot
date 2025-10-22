import numpy as np
from dataclasses import dataclass
from typing import Tuple, Callable

try:
    from scipy.signal import cont2discrete
    from scipy.linalg import solve_discrete_are
except Exception:
    cont2discrete = None
    solve_discrete_are = None

try:
    import sympy as sp
except Exception:
    sp = None


@dataclass
class LinearModel:
    A: np.ndarray  # 6x6
    B: np.ndarray  # 6x2


def _fallback_c2d(A: np.ndarray, B: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
    """Simple first-order hold discretization fallback if scipy isn't available."""
    Ad = np.eye(A.shape[0]) + A * dt
    Bd = B * dt
    return Ad, Bd


def build_linear_model(L0: float) -> LinearModel:
    """Recreate the linearized model used in matlab/sys_calc.m around upright state.

    State x = [theta, dtheta, x, dx, phi, dphi]^T
    Inputs u = [T, Tp]^T (wheel torque and body torque)
    """
    if sp is None:
        raise RuntimeError("sympy is required to build the linear model. Please install sympy.")

    # Symbols
    theta, theta1, theta2 = sp.symbols('theta theta1 theta2')
    x, x1, x2 = sp.symbols('x x1 x2')
    phi, phi1, phi2 = sp.symbols('phi phi1 phi2')
    T, Tp = sp.symbols('T Tp')

    # Parameters from sys_calc.m
    R = 0.02
    L = L0/2
    Lm = L0/2
    l = 0.0
    mw = 0.251327
    mp = 0.0248
    M = 0.325
    Iw = 5.02655e-05
    Ip = (9.66667e-07+7.9975e-06)*2
    Im = 0.000338542
    g = 9.8

    Nm = M*(x2 + (L+Lm)*(theta2*sp.cos(theta) - theta1**2*sp.sin(theta)) - l*(phi2*sp.cos(phi) - phi1**2*sp.sin(phi)))
    Pm = M*g + M*((L+Lm)*(-theta1**2*sp.cos(theta) - theta2*sp.sin(theta)) - l*(phi1**2*sp.cos(phi) + phi2*sp.sin(phi)))
    N = Nm + mp*(x2 + L*(theta2*sp.cos(theta) - theta1**2*sp.sin(theta)))
    P = Pm + mp*g + mp*L*(-theta1**2*sp.cos(theta) - theta2*sp.sin(theta))

    equ1 = x2 - (T - N*R)/(Iw/R + mw*R)
    equ2 = (P*L + Pm*Lm)*sp.sin(theta) - (N*L + Nm*Lm)*sp.cos(theta) - T + Tp - Ip*theta2
    equ3 = Tp + Nm*l*sp.cos(phi) + Pm*l*sp.sin(phi) - Im*phi2

    # Solve for second derivatives in terms of state & input (implicitly defines dynamics)
    sol = sp.solve([sp.Eq(equ1, 0), sp.Eq(equ2, 0), sp.Eq(equ3, 0)], [x2, theta2, phi2], dict=True, simplify=False)
    if not sol:
        raise RuntimeError("Failed to solve dynamics equations")
    sol = sol[0]

    # Build f(x,u): xdot = f(x,u)
    f = sp.Matrix([
        theta1,
        sol[theta2],
        x1,
        sol[x2],
        phi1,
        sol[phi2],
    ])

    X = sp.Matrix([theta, theta1, x, x1, phi, phi1])
    U = sp.Matrix([T, Tp])

    A = f.jacobian(X)
    B = f.jacobian(U)
    A0 = np.array(A.subs({theta:0, theta1:0, x:0, x1:0, phi:0, phi1:0}).evalf(), dtype=float)
    B0 = np.array(B.subs({theta:0, theta1:0, x:0, x1:0, phi:0, phi1:0}).evalf(), dtype=float)

    return LinearModel(A=A0, B=B0)


def discretize(A: np.ndarray, B: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
    if cont2discrete is not None:
        Ad, Bd, _, _, _ = cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))), dt)
        return Ad, Bd
    else:
        return _fallback_c2d(A, B, dt)


def compute_dlqr(Ad: np.ndarray, Bd: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Compute discrete-time LQR gain K via algebraic Riccati equation.
    Returns K such that u = -K x stabilizes the system.
    """
    if solve_discrete_are is None:
        raise RuntimeError("scipy.linalg.solve_discrete_are required. Install scipy>=1.8.")
    
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K


def simulate_closed_loop(Ad: np.ndarray, Bd: np.ndarray, K: np.ndarray,
                         x0: np.ndarray, steps: int) -> np.ndarray:
    """Simulate x_{k+1} = Ad x_k + Bd u_k, u = -K x_k.
    Returns array of shape (steps+1, n_state).
    """
    x_hist = np.zeros((steps+1, Ad.shape[0]))
    x = np.array(x0, dtype=float).reshape(-1)
    x_hist[0] = x
    for k in range(steps):
        u = -K @ x
        x = Ad @ x + Bd @ u
        x_hist[k+1] = x
    return x_hist
