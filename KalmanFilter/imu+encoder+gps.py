#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import math
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randn
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse
import pymap3d as pm
import pandas as pd
import math


# define
magnet_OFF = [0, 0, 0]
PI = 3.141592
msg = Odometry()

def ps(X1,Y1,X2, Y2):
    plt.ion()
    animated_plot = plt.plot(X1, Y1, 'r')[0]
    animated_plot2 = plt.plot(X1, Y1, 'b')[0]
    ps1=1

    for i in range(0, len(X1)):
        if i>30:
            animated_plot.set_xdata(X1[i-20:i])
            animated_plot.set_ydata(Y1[i-20:i])
            animated_plot2.set_xdata(X2[i-20:i])
            animated_plot2.set_ydata(Y2[i-20:i])
        else:
            animated_plot.set_xdata(X1[0:i])
            animated_plot.set_ydata(Y1[0:i])
            animated_plot2.set_xdata(X2[0:i])
            animated_plot2.set_ydata(Y2[0:i])
    plt.draw()
    plt.pause(0.01)
    if ps1 ==1:
        #print('pang')
        plt.clf()
        ps1 =0


count = 0
ax = 0
ay = 0
yaw = 0
yaw_est = 0
def IMU_Calculation(data):


    global ax
    global ay

    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y




    quaternion = (data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    msg.twist.twist.angular.x = euler[0]*180/PI # Pitch
    msg.twist.twist.angular.y = euler[1]*180/PI # Roll

    
    # calib = 260 ##################################################CALIB


    global count
    global yaw
    global yaw_est
    yaw = euler[2]*180/PI

    global calib
    yaw -= calib
    

    yaw_est = (yaw + 90)%360
    
    msg.pose.pose.orientation.z = yaw_est%360
    msg.twist.twist.angular.z = yaw%360



    acc = (data.linear_acceleration.x,data.linear_acceleration.y, data.linear_acceleration.z)
    msg.twist.twist.linear.x = float(acc[0])
    msg.twist.twist.linear.y = float(acc[1])
    msg.twist.twist.linear.z = float(acc[2])

    #################0ki##################
    msg.pose.pose.position.z = msg.twist.twist.angular.z
    if (msg.pose.pose.position.z > 180): 
        msg.pose.pose.position.z -= 360


    orientation = tf.transformations.quaternion_from_euler(msg.twist.twist.angular.y*PI/180, msg.twist.twist.angular.x*PI/180, msg.pose.pose.position.z*PI/180)

    msg.pose.pose.orientation.y = orientation[0]
    msg.pose.pose.orientation.x = orientation[1]
    msg.pose.pose.orientation.z = orientation[2]
    msg.pose.pose.orientation.w = orientation[3]

    ######################################

    count+= 1

    if count == 100:
        print("yaw:", msg.twist.twist.angular.z)
        print(msg.pose.pose.position.y)
        print(msg.pose.pose.position.x)
        pub.publish(msg)
        count = 0

vel_in = 0
vel = 0
def Vel_Calculation(data):
    global vel_in
    global vel
    vel_in = data.pose.pose.position.x
    
    if vel_in < -10:
        vel_in = vel
        print('sbsb')
    #print('vel_in:',vel_in)

def kalmankalman():
    filter = KalmanFilter(dim_x=6, dim_z=6)
    dt = 0.1   # time step 1 second

    filter.F = np.array([[1, dt, 0.5*dt**2,  0, 0, 0],
                        [0,  1, dt,  0, 0, 0],
                        [0,  0, 1, 0, 0, 0],
                        [0,  0, 0,  1, dt, 0.5*dt**2],
                        [0, 0, 0, 0, 1, dt],
                        [0, 0, 0, 0, 0, 1]])


    q = Q_discrete_white_noise(dim=3, dt=dt, var=0.001)
    #print(q)
    filter.Q = block_diag(q, q)
    #print(filter.Q)

    #print(filter.B) ?? what is this



    filter.H = np.array([[1., 0, 0, 0, 0, 0],
                        [0, 1., 0, 0, 0, 0],
                        [0, 0, 1., 0, 0, 0],
                        [0, 0, 0, 1., 0, 0],
                        [0, 0, 0, 0, 1., 0],
                        [0, 0, 0, 0, 0, 1.]])
    # print(filter.H)
    cov_sx = 0.01#0.0031360000000000003
    cov_sy = 0.01#0.0031360000000000003

    cov_vx = 0.5 #0.5
    cov_vy = 0.8 #0.5

    cov_ax = 0.5 #0.0007199025610000001
    cov_ay = 1.74 #0.0007199025610000001

    filter.R = np.array([[cov_sx, 0, 0, 0, 0, 0],
                        [0, cov_vx, 0, 0, 0, 0],
                        [0, 0, cov_ax, 0, 0, 0],
                        [0, 0, 0, cov_sy, 0, 0],
                        [0, 0, 0, 0, cov_vy, 0],
                        [0, 0.5, 0, 0, 0, cov_ay]])                                 
    # print(filter.R)


    #######INITIAL CONDITIONS########
    filter.x = np.array([[0, 0, 0, 0, 0, 0]]).T
    #print(filter.x)
    filter.P = np.eye(6) * 500.
    #print(filter.P)
    #################################

    return filter

# # 00,126.654965,37.448611
base_lat = 37.448611
base_lon = 126.654965 
base_alt = 15.4

#ilzamap
# base_lat = 37.449206 #37.4494466
# base_lon = 126.656222 #126.656523
# base_alt = 22.1 #55.476


def get_xy(lat, lon, alt):
    e, n, u = pm.geodetic2enu(lat, lon, alt, base_lat, base_lon, base_alt)

    return e, n, u


#Define Globals
raw_x = []
raw_y = []
raw_vx = []
raw_vy = []
f_x = []
f_y = []
v = []
filter = kalmankalman()


def GPS_Calculation(data):

    #Convert Postion to xy
    global v
    lon = float(data.longitude) #x
    lat = float(data.latitude) #y
    alt = float(data.altitude) #z

    e, n, u = get_xy(lat, lon, alt)
    global raw_x
    global raw_y
    raw_x.append(e) # raw x
    raw_y.append(n) # raw y




    #Set Zs
    global ax
    global ay
    global yaw_est
    global vel_in
    cos = math.cos(yaw_est)
    sin = math.sin(yaw_est)

    a_zin = math.hypot(ax, ay)
    zs = (e, vel_in*cos, a_zin*cos, n, vel_in*sin, a_zin*sin)
    
    #Apply Kalman Filter
    global filter
    filter.predict()
    filter.update(zs)

    # Position Publish
    x = float(filter.x[0]) # filtered x
    y = float(filter.x[3]) # filtered y


    # print(e,n,x, y)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y

    global f_x
    global f_y
    f_x.append(x) 
    f_y.append(y)

    # ps(f_x, f_y, raw_x, raw_y) #####################PLOT###################

    # f_x=[]
    # f_y=[]

    #Velocity Publish
    vx = float(filter.x[1]) # filtered vx
    vy = float(filter.x[4]) # filtered vy

    # msg.pose.pose.orientation.x = vx
    # msg.pose.pose.orientation.y = vy
    global vel
    vel = math.hypot(vx, vy)

    v.append(float(vel))
   # print('vel:', vel)

    #Acceleration Publish
    # a_x = float(filter.x[2])
    # a_y = float(filter.x[5])

    # msg.pose.pose.orientation.z = a_x
    # msg.pose.pose.orientation.w = a_y

    #Save Data

    # save_data_f = list(zip(f_x, f_y))
    # save_df_f = pd.DataFrame(save_data_f)
    # save_df_f.to_csv('kal3.csv', index=False, header = False)

    # save_data_v = list(v)
    # save_df_v = pd.DataFrame(save_data_v)
    # save_df_v.to_csv('vel3.csv', index=False, header = False)
    #print(len(zs))
    #print("x:",msg.pose.pose.position.x)
    #print("y:",msg.pose.pose.position.y)  
    #print("yaw:",msg.pose.pose.orientation.x)

    # pub.publish(msg)


# calib = input('calib?: ')
calib = 0
pub = rospy.Publisher('/pose', Odometry, queue_size = 1)
rospy.init_node("Position_Node",anonymous=True)
rospy.Subscriber('/gps_data/fix',NavSatFix,GPS_Calculation)
# rospy.Subscriber('/ublox_gps/fix',NavSatFix,GPS_Calculation)
rospy.Subscriber('/Vel',Odometry,Vel_Calculation)
rospy.Subscriber('/imu/data',Imu,IMU_Calculation)
rate = rospy.Rate(500)
rospy.spin()

