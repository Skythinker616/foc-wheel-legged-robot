"""Robot simulation and visualization for foc-wheel-legged-robot (Python port of MATLAB models).

Modules:
- leg: Kinematics and VMC mapping (leg_pos, leg_spd, leg_conv)
- lqr: Feedback gain K(L0) fitted polynomials
- system: Linearized dynamics and simulation utilities
- visualize: Simple matplotlib animations for leg and balance system
"""

from .leg import leg_pos, leg_jacobian, leg_spd, leg_conv
from .lqr import lqr_k
from .system import build_linear_model, discretize, compute_dlqr, simulate_closed_loop
