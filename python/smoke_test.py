import numpy as np
from robot_sim.leg import leg_pos
from robot_sim.lqr import lqr_k
from robot_sim.system import build_linear_model, discretize

pose = leg_pos(0.1, -0.1)
print('leg_pos:', pose)

K = lqr_k(0.07)
print('K shape:', K.shape, 'first row sum:', float(K[0].sum()))

model = build_linear_model(0.07)
Ad, Bd = discretize(model.A, model.B, 0.005)
print('A/B shapes:', model.A.shape, model.B.shape)
print('Ad/Bd shapes:', Ad.shape, Bd.shape)
print('Ad first row:', Ad[0])
