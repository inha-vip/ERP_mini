import csv
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
#'''
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../CubicSpline/")

import cubic_spline_planner
#'''
# try:
#     from quintic_polynomials_planner import QuinticPolynomial
#     import cubic_spline_planner
# except ImportError:
#     raise


# tx, ty, tyaw, tk, ts = [], [], [], [], []
# with open('/home/vip/Desktop/8888.csv') as csv_file:
#     csv_reader = csv.DictReader(csv_file)
#     for next_r in csv_reader:
#         tx.append(float(next_r['x']))
#         ty.append(float(next_r['y']))
#         tyaw.append(float(next_r['yaw']))
#         tk.append(float(next_r['k']))
#         ts.append(float(next_r['s']))
# plt.scatter(tx, ty)
# plt.grid(True)
# plt.axis([-100, 100, -100, 100])

# tx, ty, tyaw, tk, ts = [], [], [], [], []
# with open('/home/vip/Desktop/9999.csv') as csv_file:
#     csv_reader = csv.DictReader(csv_file)
#     for next_r in csv_reader:
#         tx.append(float(next_r['x']))
#         ty.append(float(next_r['y']))
#         tyaw.append(float(next_r['yaw']))
#         tk.append(float(next_r['k']))
#         ts.append(float(next_r['s']))

# plt.scatter(tx,ty)
# tx, ty, tyaw, tk, ts = [], [], [], [], []
# with open('/home/vip/Desktop/7777.csv') as csv_file:
#     csv_reader = csv.DictReader(csv_file)
#     for next_r in csv_reader:
#         tx.append(float(next_r['x']))
#         ty.append(float(next_r['y']))
#         tyaw.append(float(next_r['yaw']))
#         tk.append(float(next_r['k']))
#         ts.append(float(next_r['s']))

# plt.scatter(tx,ty)

# plt.show()

def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


wx = [33.0, 23.0]
wy = [29.0, 29.0]
tx, ty, tyaw, tk, ts =  generate_target_course(wx, wy)

plt.scatter(tx, ty)
plt.show()

f = open('/home/vip/Desktop/5555.csv', 'w', newline='')
wr = csv.writer(f)
wr.writerow(['x', 'y', 'yaw', 'k', 's'])
for i in range(len(tx)):
    wr.writerow([tx[i], ty[i], tyaw[i], tk[i], ts[i]])

f.close