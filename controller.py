#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import hypot
from math import atan2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
# import LPP
import sys

sys.path.append("./map/")

try:
    import GPP
except:
    raise


class Controller:
    def __init__(self,  master):
        self.control_data = master.control_data
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.GPP_array_msg = PointCloud()
        self.GPP_pt_msg = Point32()
        self.GPP_array_msg.points = []
        self.vel_msg = Odometry()
        self.gpp_pub = rospy.Publisher('/GPP', PointCloud, queue_size=1)
        self.vel_pub = rospy.Publisher('/Vel', Odometry, queue_size=1)

        # self.bring_tmp_map()
        
    def get_xy(self,  lat,  lon,  alt):
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        return e,  n
    
    '''
    #학관앞에서 gpp 안하고 그냥 대충 해볼때 쓰던 함수임. GPP 하면 안씀
    def bring_tmp_map(self):
        # with open('/home/vip/Desktop/ERP_mini/save.csv',  mode ='r') as csv_file:
        with open('/home/junhyeok/Desktop/save.csv',  mode ='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for next_r in csv_reader:
                lon = float(next_r['field.longitude'])
                lat = float(next_r['field.latitude'])
                x,  y = self.get_xy(lat,  lon,  0.9)
                self.path_x.append(x)
                self.path_y.append(y)
    '''


        # 이 함수도 lpp가 되면 안 쓰이고 바뀔 수도 
    def closest_pt(self,  x, y, yaw):
        min_dis = 99999
        dis_list = []
        for i in range(len(self.path_x)):
            dis = ((self.path_x[i]-x)**2 +(self.path_y[i]-y)**2)**0.5
            dis_list.append(dis)
        
        self.control_data['target_idx'] = dis_list.index(min(dis_list))
        self.control_data['first_check'] = False
            
            #if dis <= min_dis:
              #  min_dis = dis
                #self.control_data['target_idx'] = i
                #self.control_data['first_check'] = False
             
    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead):
        target_x = self.path_x[self.control_data['target_idx']]
        target_y = self.path_y[self.control_data['target_idx']]
        
        # print('target_x:',  target_x,  'target_y:',  target_y)
        
        dis = hypot(target_x - cur_x,  target_y - cur_y)

        if dis <= self.control_data['look_ahead']:
            print('########################short:',  self.control_data['target_idx'])
            self.control_data['target_idx'] += 1
            return self.cal_steering(cur_x, cur_y, cur_yaw, look_ahead)
        else : 
            # print('hit the target:',  dis)
            return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
             
    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        # pure pursuit 계산되는 부분 
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        # print('cur_x:',  cur_x ,  'cur_y',  cur_y,  'target_x:',  target_x,  'target_y:',  target_y,  'atan2((target_y - cur_y), (target_x - cur_x):',  tmp_th)
        # print('-'*30)
        #print('original target theta:',  tmp_th)
        #print('cur_yaw:',  cur_yaw)
        
        tmp_th = tmp_th%360
        #print('360 target theta:',  tmp_th)
        
        
        if (tmp_th > 90) and (tmp_th <= 360):
            tmp_th -= 90
        else:
            tmp_th += 270
        #print('changed 360 target theta:',  tmp_th)
        #print('cur_yaw:',  cur_yaw,  'target_angle:',  tmp_th)
        alpha =  cur_yaw - tmp_th  
        # print('alpha:',  alpha)
        if abs(alpha)>180:
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        # print('alpha2:',  alpha)        
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        # print('modified alpha:',  alpha)
        
        delta = degrees(atan2(2*self.control_data['WB']*sin(radians(alpha))/self.control_data['look_ahead'],1))
        #print('delta:',  delta)
        
        #test = tmp_th - cur_yaw
        
        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        
        
        # print("delta2 : ", delta)
        if abs(delta)>30:
            if delta > 0:
                #print('del:',  '1900')
                return 1900
            else :
                #print('del:',  '1200' )
                return 1200
        else :
            delta = 1550 + (350/30)*delta
            #print('del:',  delta)
            return int(delta)

    def getObstacleMsg(self, msg):
        #라이다에서 장애물 받아오는 콜백 함수인데.. LPP는 연구중... ㅠ 
        #추후에 테스트 한다면 지금 한번에 받아오는 200개 가량의 장애물이 잘 처리가 되는지가 관건일듯.. 
        ob = []
        for pt in msg.points:
            ob.append([pt.x, pt.y])

        ob = np.array(ob)

        #print(ob)
        #print(len(ob))
 

    def getOdoMsg(self,  msg):
        #print('get msg')
        cur_lon    = msg.pose.pose.position.x
        cur_lat    = msg.pose.pose.position.y
        self.control_data['cur_yaw']  = msg.twist.twist.angular.z
        
        #print('lon:',  cur_lon,  'lat:',  cur_lat, 'yaw:',  self.control_data['cur_yaw'])
        
        self.control_data['cur_x'], self.control_data['cur_y']  = self.get_xy(cur_lat,  cur_lon,  0.9)
        
        
        
        # 맨 처음 gps imu 메세지 받았을 때만 실행 됨 -> 전체 경로 받아옴.
        if self.control_data['first_check'] :
            print('this is first time')
            print( 'cur_x:',  self.control_data['cur_x'], 'cur_y:',  self.control_data['cur_y'],  'cur_yaw:',  self.control_data['cur_yaw'])
            gp = GPP.Global_planner(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
            gp.lane_info()
            self.path_x, self.path_y, self.path_yaw ,_ ,_ = gp.planning(gp.start_x, gp.start_y, gp.start_yaw, gp.goal_x, gp.goal_y)

            # print('path_x:',  self.path_x)
            self.control_data['gpp_check'] = False
            self.control_data['first_check'] = False

            self.closest_pt(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
            
            # self.control_data['GPP_array_msg'].points = []
            # for i in range(len(self.path_x)):
            #     self.control_data['GPP_pt_msg'].x = self.path_x[i]
            #     self.control_data['GPP_pt_msg'].y = self.path_y[i]
            #     self.control_data['GPP_pt_msg'].z = self.path_yaw[i]
            #     self.control_data['GPP_array_msg'].points.append(self.control_data['GPP_pt_msg'])
                

            # temptemp= self.control_data['GPP_array_msg']
            # self.control_data['gpp_pub'].publish(temptemp)

            for i in range(len(self.path_x)):
                self.GPP_pt_msg.x = 1 #self.path_x[i]
                self.GPP_pt_msg.y = 1 #self.path_y[i]
                self.GPP_pt_msg.z = 1 #self.path_yaw[i]
                self.GPP_array_msg.points.append(self.GPP_pt_msg)
            print('put msg complete')
            self.gpp_pub.publish(self.GPP_array_msg)    
            print('send the msg')
            '''
            bring GPP map
            a map exist from now on
            '''
        # print( 'x:',  self.control_data['cur_x'], 'y:',  self.control_data['cur_y'],  'yaw:',  self.control_data['cur_yaw'])
        #print('index:',  self.control_data['target_idx'])
        print('speed(m/s):',  self.control_data['speed'], 'speed(km/h):', self.control_data['speed']*3.6)
        self.control_data['steering']  = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'],  self.control_data['look_ahead'])
        self.vel_msg.pose.pose.position.x  =  self.control_data['speed']
        self.vel_pub.publish(self.vel_msg)
        #self.control_data['steering'] = 1900            

    def run(self):
        
        #print(self.control_data)
        #print('controller')
        rospy.Subscriber("/pose", Odometry, self.getOdoMsg)
        # rospy.Subscriber('Obs', Obs, self.getObstacleMsg)