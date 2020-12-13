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



def GPS_Position_Publish(data):
    lat = float(data.latitude)
    lon = float(data.longitude)
    inin = input("name? :")
    print(inin, lat, lon)





#main()

rospy.init_node("Nogada_Node",anonymous=True)
rospy.Subscriber('/ublox_gps/fix',NavSatFix,GPS_Position_Publish)
rate = rospy.Rate(500)
rospy.spin()