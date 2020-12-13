import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time

dt = 0.1

f = KalmanFilter(dim_x = 2, dim_z = 1)

f.x = np.array([[2.], [0.]])
f.F = np.array([[1.,dt],[0.,1.]])

f.H = np.array([[1.,0.,]])
f.P *= 1000.
f.R = 5
f.Q = Q_discrete_white_noise(2, dt, .1)
#print(f.Q.size)
i = 1
z=2
while True:
    f.predict()
    f.update(z)
    
    x = f.x
    print(x)
    print(i)
    i +=1
    z *=2
