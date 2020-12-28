#!/usr/bin/env python
#-*- coding:utf-8 -*-
import threading

import rospy
from sender import Sender
from controller import Controller
from getvelocity import GetVelocity
import signal
import serial
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
# from testtest import Sibal


# 'GPP_pt_msg': Point32(),  'GPP_array_msg': PointCloud(), 
# 'gpp_pub': rospy.Publisher('/GPP', PointCloud,  queue_size = 1  ),

class Master:
    def __init__(self):
        rospy.init_node('manager', anonymous=False)
        signal.signal(signal.SIGINT, self.handler)
        print('master on')
        
        #공유 데이터 => sender.py , controller.py 랑 실시간으로 값 공유 됨.
        self.control_data = {'direction':0, 'target_speed': 600 ,'speed': 0.0, 'steering':0, 
        'look_ahead': 2.0, 'target_idx': 0, 'WB': 1, 'gpp_check':True, 'line':[],  
        'exitThread': False,  'cur_idx':0, 'LPP_mode': False, 
        'base_lat': 37.448611,  'base_lon': 126.654965,  'base_alt': 15.400, 
        'ser':  serial.Serial('/dev/ttyUSB1',115200,  timeout=0), 
        'cur_x':0.0, 'cur_y':0.0, 'cur_yaw':0.0, 'first_check': True  }
        
        self.sender = Sender(self)
        self.controller = Controller(self)
        self.getvelocity = GetVelocity(self)
        #self.sibal = Sibal(self)

        #sender, controller 스레드 돌림.
        th_sender = threading.Thread(target=self.sender.run, args=())
        th_controller = threading.Thread(target=self.controller.run, args=())
        th_getvelocity = threading.Thread(target=self.getvelocity.run_vel, args=( ))  #self.control_data['ser']
        #th_sibal = threading.Thread(target=self.sibal.sibal_run, args=())

        th_sender.start()
        th_controller.start()
        th_getvelocity.start()
        #th_sibal.start()

        th_sender.join()
        #th_controller.join()
        #th_getvelocity.join()
        #th_sibal.join()
    
        #rospy.Subscriber("/pose", Odometry, self.show)
        rospy.Rate(1)
        rospy.spin()

    def show():
        control_data['cur_x']    = msg.pose.pose.position.y
        print(self.control_data['cur_x'])

    def handler(self,  signum, frame):
        # self.control_data['exitThread'] = True
        exit()


if __name__ == "__main__":
	Master()
