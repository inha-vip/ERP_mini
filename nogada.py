#!/usr/bin/env python
import rospy
import math
import tf
import pymap3d as pm
import keyboard as ky

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
lat = 0
lon = 0
seq = 0


def GPS_Position_Publish(data):

    global lat
    global lon
    global seq
    lat = float(data.latitude)
    lon = float(data.longitude)
    seq = data.header.seq



while True:

    
    inin = input("name? :")

    for i in range(0, 20):
        rospy.Subscriber('/gps_data/fix',NavSatFix,GPS_Position_Publish)

    print(inin, seq, lat, lon)




#main()
rospy.init_node("Nogada_Node",anonymous=True)
# rospy.Subscriber('/gps_data/fix',NavSatFix,GPS_Position_Publish)
rate = rospy.Rate(500)
rospy.spin()



        
