#!/usr/bin/env python
import rospy
import math
import tf
import pymap3d as pm

import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# define
magnet_OFF = [0, 0, 0]
PI = 3.141592
msg = Odometry()



#Initial Vaules
base_lat = 37.448611
base_lon = 126.654965
base_alt = 15.400

alpha = 0.7
initial = True


def LowPassFilter(alpha, est, measure):
    new_est = alpha*est + (1-alpha)*measure
    return new_est
    

def Position_Calculation(data):
    lat = float(data.latitude)
    lon = float(data.longitude)
    x, y, u = pm.geodetic2enu(lat, lon, base_alt, base_lat, base_lon, base_alt)
    
    #while initial is True:
    #    x_est = LowPassFilter(alpha, 0, x)
    #    initial = False
#
    #x_est = LowPassFilter(alpha, x_est, x)
    x_est = 1
    y_est = 1
    print("x", x)
    return x_est, y_est

    
   # print(x_est)
    
def GPS_Position_Publish(data):
    calc = Position_Calculation(data)
    msg.pose.pose.position.x = float(calc[0])
    msg.pose.pose.position.y = float(calc[1])

    lat = float(data.latitude)
    lon = float(data.longitude)
    print(lat)
    print(lon)
    print(calc[0])
    print("succc")
    pub.publish(msg)



def IMU_Calculation(data):
    quaternion = (data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    msg.twist.twist.angular.x = euler[0]*180/PI # Pitch
    msg.twist.twist.angular.y = euler[1]*180/PI # Roll
    msg.twist.twist.angular.z = euler[2]*180/PI # Yaw
    
    # Angle detail control
    msg.twist.twist.angular.z -= 260
    msg.twist.twist.angular.z = msg.twist.twist.angular.z%360


    
    #print("yaw:", msg.twist.twist.angular.z)


def velocity_estimate(data):
    encoder = data.pose.pose.position.x
    #print(encoder)

    

#main()



pub = rospy.Publisher('/pose', Odometry, queue_size = 1)
rospy.init_node("Position_Node",anonymous=True)
rospy.Subscriber('/ublox_gps/fix',NavSatFix,GPS_Position_Publish)
#rospy.Subscriber('/ublox_gps/fix',NavSatFix,Position_Calculation)
#rospy.Subscriber('/imu/data',Imu,IMU_Calculation)
#rospy.Subscriber('/Vel', Odometry, velocity_estimate) 

rate = rospy.Rate(500)
rospy.spin()