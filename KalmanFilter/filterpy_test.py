import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


dt = 0.1

f = KalmanFilter(dim_x = 2, dim_z = 1)

f.x = np.array([[2.], [0.]])
f.F = np.array([[1.,1.],[0.,1.]])

f.H = np.array([[1.,0.,]])
f.P *= 1000.and
f.R = 5
f.Q = Q_discrete_white_noise(2, dt, .1)

while True:
    f.predict()
    f.update(get_some_meas
