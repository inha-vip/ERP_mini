import numpy as np
#import matplotlib.pyplot as plt
import copy
from math import cos
from math import sin
from math import pi
from math import atan2
from math import hypot
import matplotlib.pyplot as plt
import sys
import os
#'''
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../QuinticPolynomialsPlanner/")
#'''
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../CubicSpline/")
from quintic_polynomials_planner import QuinticPolynomial
#'''
try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise
#'''
#SIM_LOOP = 10

# Parameter
# MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
# MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
# MAX_CURVATURE = 1.0  # maximum curvature [1/m]
# MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
# D_ROAD_W = 1.0  # road width sampling length [m]
# DT = 0.2  # time tick [s]
# MAX_T = 5.0  # max prediction time [m]
# MIN_T = 4.0  # min prediction time [m]
# TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
# D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
# N_S_SAMPLE = 1  # sampling number of target speed
# ROBOT_RADIUS = 2.0  # robot radius [m]

MAX_SPEED = 20.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 3  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 2.5  # maximum road width [m]
D_ROAD_W = 1  # road width sampling length [m]
DT = 1.0  # time tick [s]
MAX_T = 5.0  # max prediction time [m]
MIN_T = 2.0  # min prediction time [m]
# TARGET_SPEED = 0.1 / 3.6  # target speed [m/s]
D_T_S = 1.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 0.7  # robot radius [m]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, target_speed):
    frenet_paths = []
    # target_speed = 2*0.1*math.pi*target_speed/152/2
    target_speed = 12.0/3.6
    # print('target speed:', target_speed)
    # n = data
        # N= 152
        # t = 1
        
        # w = 2*math.pi*n/N/t
        
        # r = 0.1 #m
        # v = r*w*3.6 #km/h
        # output = v

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(target_speed - D_T_S * N_S_SAMPLE,
                                target_speed + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (target_speed - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths

def sd_to_xy(s, d, tx, ty, tyaw, tk, ts):
    #ts = ts.tolist()
    min_val = 99999
    min_idx = 0
    for i in range(len(ts)):
        if abs(ts[i] - s) < min_val:
            min_idx = i
            min_val = abs(ts[i] - s)

    s_idx = min_idx
    #s_idx = ts.index(s)
    x = tx[s_idx]
    y = ty[s_idx]
    
    #pi = 3.14
    #rx = x + d* math.cos((math.pi/2) + tyaw[s_idx])
    #ry = y + d* math.sin((math.pi/2) + tyaw[s_idx])  
    rx = x + d * cos(3.14/2 + tyaw[s_idx])
    ry = y + d * sin(3.14/2 + tyaw[s_idx])
    #print("rx, ry", rx, ry)
    #print("d is ", d)
    #print("d is ", d)

    #d= round(d, 1)
    #print("round d is", d)
    #rx = d
    #ry = d
    #rx, ry = 0, 0
    return rx, ry


def calc_global_paths(fplist, tx, ty, tyaw, tc, ts):
    fp = []
    for fp in fplist:
        #fp = fplist[0]
        # calc global positions
        for i in range(len(fp.s)):
            fx, fy = sd_to_xy(fp.s[i], fp.d[i], tx, ty, tyaw, tc, ts)
            # fx, fy = 0, 0
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(atan2(dy, dx))
            fp.ds.append(hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])
        #print("okok")
        # calc curvature
        for i in range(len(fp.yaw) - 1):
            #print("okok2")
            #fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])


        # print('dis:' , math.hypot(fp.x[i+1]-fp.x[i], fp.y[i+1]- fp.y[i]))

    return fplist


def check_collision(fp, ob):
    # print('ob:', ob)
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]
        
        print('d:', d)

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])
        print('collision:', collision)

        if collision:
            return False

    return True

def calc_coll(fp, ob):

    dist = 100

    for i in range(len(fp.x)-1):
        for j in range(len(ob[:, 0])):            
            son = (fp.y[i+1] - fp.y[i])*(ob[j][0] -fp.x[i]) + (fp.x[i+1]-fp.x[i])*(fp.y[i]-ob[j][1])
            mom = hypot(fp.y[i+1] - fp.y[i], fp.x[i+1] - fp.x[i])
            # son = (py2-py1)*(ob[i][0]-px1) + (px2-px1)*(py1-ob[i][1])
            # mom = hypot(py2-py1, px2-px1)
            dist = abs(son) / mom 
            
            # print(i, "th dist: ", dist)

            if dist < 0.1:
                print("COLL####################")
                print("COLL####################")
                print("COLL####################")
                print("COLL####################")
                return True

    if dist >= 0.1:
        return False


def check_paths(fplist, ob):
    ok_ind = []
    # for i, _ in enumerate(fplist):
    for i in range(len(fplist)):
        # print("fplist.x: ", fplist[i].x)
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            # print('number', i, 'path eliminated by MAX SPEED')
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            # print('number', i, 'path eliminated by MAX accel')
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            # print('number', i, 'path eliminated by MAX CURVATURE')
            continue
        # elif not check_collision(fplist[i], ob):
        #     print('number', i, 'path eliminated by COLLISION')
        #     continue

        elif calc_coll(fplist[i], ob):
            print("obstacle")
            print('number', i, 'path eliminated by LINE collision')
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]

count = 0
tmp_list= []

def frenet_optimal_planning(s0, c_speed, c_d, c_d_d, c_d_dd, ob, tx, ty, tyaw, tc, ts, target_speed):
    
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, target_speed)
    #print('x list0:', fplist[0].x)
    fplist = calc_global_paths(fplist, tx, ty, tyaw, tc, ts)
    print('number of fplist:', len(fplist))
    # print("ob is ", ob)
    # plt.plot([45.63 , -45.5451172257],[98.47, -95.4670722654])
    # plt.plot([6.78591132663 , -6.06284527727],[15.8453863844, -11.4849896419])
    # plt.plot([33.0, 29.0],[27.0, 29.0])
    # plt.plot(tx, ty)
    global tmp_list
    fplist = check_paths(fplist, ob)

    # if len(fplist) != 0:
    #     print("not none")
    #     fplist = fplist2
    # # print('ob x:',ob[:,0])
    # plt.scatter(ob[:,0], ob[:,1], marker='o')
    # if len(fplist)!=0:
    #     print('original path')
    #     tmp_list = fplist
    # else:
    #     print('previous path')
    #     fplist = tmp_list
    
    

    # for i in fplist:
    #     plt.scatter(i.x,i.y)
    #     # tmp_ob = ob.tolist()
    #     # plt.scatter(tmp_ob[0][0], tmp_ob[0][1], marker='x')
    #     # plt.scatter(tmp_ob[1][0], tmp_ob[1][1], marker='x')
    #     plt.text(i.x[0], i.y[0], 'start')
    #     # plt.text(self.control_data['cur_x'], self.control_data['cur_y'], 'cur_point')
    #     # plt.text(i.x[-1], i.y[-1], 'goal')
    #     plt.grid
    # plt.show()

    #print('x list2:', fplist[0].x)

    # global count
    # find minimum cost path
    min_cost = float("inf")
    best_path = fplist

    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    # plt.plot(best_path.x, best_path.y)
    # plt.text(best_path.x[0], best_path.y[0], 's')
    # plt.show()

    return best_path.x , best_path.y , best_path.c ,best_path.ds
    