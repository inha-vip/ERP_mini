#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

import pymap3d
import csv
import rospy
# from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obs
import time
import numpy as np
import matplotlib.pyplot as plt
from math import radians
from math import degrees
from math import sin
from math import cos
from math import hypot
from math import atan2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import LPP
import sys
import math

sys.path.append("./map/")

try:
    import GPP
except:
    raise


class Controller:
    def __init__(self,  master):
        self.control_data = master.control_data
        self.g_path_x = []
        self.g_path_y = []
        self.g_path_yaw = []
        self.g_path_k = []
        self.g_path_s = []
        
        self.l_path_x = []
        self.l_path_y = []
        self.l_path_yaw = []
        self.l_path_k = []
        self.l_path_s = []
        self.local_idx = 0

        self.goal_x = 9999
        self.goal_y = 9999
        # self.goal_idx = 0
        self.goal_lane = ''
    
        self.ob = np.array([])
        self.start_t = 0

        self.s = 0
        self.c_d = 0
        self.c_d_d = 0
        self.c_d_dd = 0
        
        self.old_c_d =0 
        self.old_c_d_d = 0 
    
        self.GPP_array_msg = PointCloud()
        self.LPP_array_msg = PointCloud()
        self.OBB_array_msg = PointCloud()
        self.GPP_pt_msg = Point32()

        self.GPP_array_msg.points = []
        self.LPP_array_msg.points = []
        self.OBB_array_msg.points = []
        # self.vel_msg = Odometry()
        self.gpp_pub = rospy.Publisher('/GPP', PointCloud, queue_size=1)
        # self.vel_pub = rospy.Publisher('/Vel', Odometry, queue_size=1)
        self.lpp_pub = rospy.Publisher('/LPP', PointCloud, queue_size=1)
        self.ob_pub = rospy.Publisher('/OBB', PointCloud, queue_size=1)

        self.end_pt = False
        self.LPPsw = False
        self.tick = False
        self.test_bool = False
        self.LPP_count = 0


    


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
                self.g_path_x.append(x)
                self.g_path_y.append(y)
    '''

    
    # def ps(self, X1,Y1,X2, Y2):
    #     plt.ion()
    #     # animated_plot = plt.plot(X1, Y1, 'r')[0]
    #     # animated_plot2 = plt.plot(X2, Y2, 'b')[0]
    #     ps1=1
    #     index=0
    #     D=100000
    #     for j in range(0, len(X1)):
    #         dx=math.pow((X1[j]-X2[0]),2)
    #         dy=math.pow((Y1[j]-Y2[0]),2)
    #         D1=math.sqrt(dx+dy)
    #         if D1<D:
    #             D=D1
    #             index=j
            
    #     for i in range(0, len(X2)):
    #         '''
    #         if i>500:
    #             animated_plot.set_xdata(X1[i-20:i])
    #             animated_plot.set_ydata(Y1[i-20:i])
    #             animated_plot2.set_xdata(X2[0:i])
    #             animated_plot2.set_ydata(Y2[0:i])
    #             print('pang')
    #         else:
    #             animated_plot.set_xdata(X1[0:i])
    #             animated_plot.set_ydata(Y1[0:i])
    #             animated_plot.set_xdata(X1[0:i])
    #             animated_plot.set_ydata(Y1[0:i])
    #         '''

    #         # animated_plot2.set_xdata(X2[0:i])
    #         # animated_plot2.set_ydata(Y2[0:i])
    #         #animated_plot2.set_xdata(X1[index:index+i])
    #         #animated_plot2.set_ydata(Y1[index:index+i])
            
    #         #print('pang1')
    #     plt.draw()
    #     plt.pause(0.1)
    #     if ps1 ==1:
    #         #print('pang')
    #         plt.clf()
    #         ps1 =0

    def test_path(self):
        tx, ty, tyaw, tk, ts = [], [], [], [], []
        with open('/home/vip/Desktop/6666.csv') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for next_r in csv_reader:
                tx.append(float(next_r['x']))
                ty.append(float(next_r['y']))
                tyaw.append(float(next_r['yaw']))
                tk.append(float(next_r['k']))
                ts.append(float(next_r['s']))

        return tx, ty, tyaw, tk, ts

    def xytosd(self):
        tmp_list = []
        c_idx = self.control_data['cur_idx']

        # print('xytosd cur idx',self.control_data['cur_idx'])
        #cnt = 0
        for i in range(self.control_data['cur_idx'], len(self.g_path_s)):
            # self.control_data['cur_x'] += 10
            #self.control_data['cur_x'] = self.l_path_x[c_idx]
            #self.control_data['cur_y'] = self.l_path_y[c_idx]
            dis = hypot(self.control_data['cur_x']- self.g_path_x[i] , self.control_data['cur_y']- self.g_path_y[i])
            #print("dis", cnt, i, dis)
            #cnt += 1
            tmp_list.append(dis)

        #print("temp_min_idx, ", temp_min_idx)
        # print("tmplist", tmp_list)
        self.control_data['cur_idx'] = tmp_list.index(min(tmp_list)) + self.control_data['cur_idx']
        #self.control_data['cur_idx'] += 1
        #print("min tmp idx, ", tmp_list.index(min(tmp_list)))
        #self.control_data['cur_idx'] = tmp_list.index(min(tmp_list))
        s = self.g_path_s[self.control_data['cur_idx']]
        d = min(tmp_list)
        #d = min_dis
        #print("g_path_s is", self.g_path_s)
        #print("s, d is", s, d)
        
        tmp_idx = self.control_data['cur_idx']
        
        if tmp_idx+1 != len(self.g_path_x):
            a = np.array([ self.control_data['cur_x'] - self.g_path_x[tmp_idx] , self.control_data['cur_y'] - self.g_path_y[tmp_idx] ])
            b = np.array([ self.g_path_x[tmp_idx+1] - self.g_path_x[tmp_idx] , self.g_path_y[tmp_idx+1] - self.g_path_y[tmp_idx] ])
            if np.cross(b, a) > 0:
                return s, d
            else:
                return s, -d

        else:
            a = np.array([self.control_data['cur_x'] - self.g_path_x[tmp_idx]] , [self.control_data['cur_y'] - self.g_path_y[tmp_idx]])
            b = np.array([self.g_path_x[tmp_idx] - self.g_path_x[tmp_idx-1] , self.g_path_y[tmp_idx] - self.g_path_y[tmp_idx-1] ])
            if np.cross(b, a) > 0:
                return s, d
            else:
                return s, -d
        

        




        # 이 함수도 lpp가 되면 안 쓰이고 바뀔 수도 
    def closest_pt(self, x, y, yaw):
        min_dis = 99999
        dis_list = []
        for i in range(len(self.g_path_x)):
            dis = ((self.g_path_x[i]-x)**2 +(self.g_path_y[i]-y)**2)**0.5
            dis_list.append(dis)

        # print('dis_list:',dis_list)
        # self.control_data['target_idx'] = dis_list.index(min(dis_list))
        # self.control_data['cur_idx'] = dis_list.index(min(dis_list))
        #print('##############################################')
        #print(' closest idx: ', self.control_data['cur_idx'])
        #print('##############################################')
        # self.control_data['first_check'] = False
        peibr("first check is", self.control_data['first_check'])
            
            #if dis <= min_dis:
              #  min_dis = dis
                #self.control_data['target_idx'] = i
                #self.control_data['first_check'] = False

    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead, LPP_mode):
        while(True) :
            if LPP_mode:
                self.local_idx = 1
                target_x = self.l_path_x[self.local_idx]
                target_y = self.l_path_y[self.local_idx]
            else:
                target_x = self.g_path_x[self.control_data['target_idx']]
                target_y = self.g_path_y[self.control_data['target_idx']]
            
            # target_x = self.l_path_x[-2]
            # target_y = self.l_path_y[-2]
            dis = hypot(target_x - cur_x,  target_y - cur_y)
            # print('check : ', dis)
            # print('check2: ', self.local_idx)
            # print('check3: ', dis <= self.control_data['look_ahead'])
            return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)

            if dis > self.control_data['look_ahead']:
                return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
            
            else :
                if LPP_mode:
                    self.local_idx += 1
                else:
                    self.control_data['target_idx'] += 1

    # def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead, LPP_mode):
    #     if LPP_mode:
    #         # print('-------------------------------')
    #         # print('LPP mode!!!!!!!!!!!!!!!!!!!!!!!')
    #         # print('-------------------------------')
            
    #         target_x = self.l_path_x[self.local_idx]
    #         target_y = self.l_path_y[self.local_idx]
    #     else:
            
    #         # print('GPP mode!!!!!!!!!!!!!!!!!!!!!!!!')
    #         target_x = self.g_path_x[self.control_data['target_idx']]
    #         target_y = self.g_path_y[self.control_data['target_idx']]
        
    #     look_x = [target_x , cur_x]
    #     look_y = [target_y , cur_y]
    #     plt.plot(look_x, look_x, 'g')
    #     # plt.scatter(target_x, target_y, marker='o')
    #     dis = hypot(target_x - cur_x,  target_y - cur_y)
        
    #     if dis <= self.control_data['look_ahead']:
    #         # print('########################short:',  self.control_data['target_idx'])
    #         if LPP_mode:
    #             #print('local index up!')
    #             self.local_idx += 1
    #         else:
    #             #print('global index up!')
    #             self.control_data['target_idx'] += 1
    #         return self.cal_steering(cur_x, cur_y, cur_yaw, look_ahead, LPP_mode)
    #     else : 
    #         # print('hit the target:',  dis)
    #         #print('target_x:',  target_x,  'target_y:',  target_y)
    #         return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
        

    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        # pure pursuit 계산되는 부분 
        self.local_idx = 1
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        # print('cur_x:',  cur_x ,  'cur_y',  cur_y,  'target_x:',  target_x,  'target_y:',  target_y,  'atan2((target_y - cur_y), (target_x - cur_x):',  tmp_th)
        # print('-'*30)
        #print('original target theta:',  tmp_th)
        #print('cur_yaw:',  cur_yaw)
        # arrow_x = [target_x, cur_x]
        # arrow_y = [target_y, cur_y]
        # plt.plot(arrow_x, arrow_y)
        # plt.text(target_x, target_y, 'goal')
        # plt.text(0,0, cur_yaw)
        cur_yaw = cur_yaw%360
        if (cur_yaw > 0) and (cur_yaw <= 270):
            cur_yaw += 90
        else:
            cur_yaw -= 270

        # look_x = [10*cos(radians(cur_yaw))+cur_x , cur_x]
        # look_y = [10*sin(radians(cur_yaw))+cur_y , cur_y]
        # plt.plot(look_x, look_y)
        # plt.text(10*cos(radians(cur_yaw))+cur_x , 10*sin(radians(cur_yaw))+cur_y, cur_yaw)
        # plt.text(10,10, cur_yaw)
        
        # plt.text(0,0, tmp_th)

        # tmp_th = tmp_th%360
        #print('360 target theta:',  tmp_th)
        
        
        # if (tmp_th > 90) and (tmp_th <= 360):
        #     tmp_th -= 90
        # else:
        #     tmp_th += 270
        # plt.scatter(target_x, target_y, marker='x')
        # plt.text(target_x, target_y, tmp_th)
        #print('changed 360 target theta:',  tmp_th)
        # print('-'*30)
        # print('cur_x:', cur_x, 'cur_y: ', cur_y, 'target_x:', target_x, 'target_y:', target_y)
        # print('dx:', target_x - cur_x , 'dy:', target_y - cur_y)
        # print('cur_yaw:',  cur_yaw,  'target_angle:',  tmp_th)
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
         
        # print('delta:',  delta)
        
        # test = tmp_th - cur_yaw
        
        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        
        # print("delta2 : ", delta)
        if abs(delta)>30:
            if delta > 0:
                print('max del:',  '1900')
                return 1900
            else :
                print('min del:',  '1200' )
                return 1200
        else :
            delta = 1550 + (350/30)*delta
            print('del:',  delta)
            return int(delta)

    def getGoalpt(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        print('goal_x:', self.goal_x)
        print('goal_y:', self.goal_y)
        self.goal_lane = msg.text
        print('goal_lane:', self.goal_lane)
        self.control_data['first_check'] = True
        self.control_data['gpp_check'] = True

    def getObstacleMsg(self, msg): 
        # print('get lidar')
        self.ob = []
        for pt in msg.points:
            lidar_d = hypot(pt.x, pt.y )

            lidar_yaw = degrees(atan2(pt.y, pt.x))
            lidar_yaw = lidar_yaw%360
            if self.control_data['cur_yaw'] > 0 and self.control_data['cur_yaw'] <270:
                tmp_yaw = self.control_data['cur_yaw'] + 90
            else:
                tmp_yaw = self.control_data['cur_yaw'] - 270

            # print('datax : ', pt.x)
            # print('datay : ', pt.y)
            
            lidar_x = self.control_data['cur_x'] + lidar_d * cos(radians(tmp_yaw + lidar_yaw - 90))
            lidar_y = self.control_data['cur_y'] + lidar_d * sin(radians(tmp_yaw + lidar_yaw - 90))
            # print('datax2 : ', self.control_data['cur_x'])
            # print('datay2 : ', self.control_data['cur_y'])
            self.ob.append([lidar_x , lidar_y])

        for i in range(0, len(self.ob)):
            self.OBB_pt_msg = Point32()
            self.OBB_pt_msg.x = self.ob[i][0]
            self.OBB_pt_msg.y = self.ob[i][1]
            # self.GPP_pt_msg.z = self.g_path_yaw[i]
            self.OBB_array_msg.points.append(self.OBB_pt_msg)               
        
        self.ob_pub.publish(self.OBB_array_msg)       
    
        self.ob = np.array(self.ob)
        print(self.ob)
        # print(len(self.ob))
        # self.ob = np.array([[17.6, -8.4]])
        # self.ob = np.array([[1.0, -1.0]])
        #print('cur_x:', self.control_data['cur_x'], 'cur_y:' , self.control_data['cur_y'])    
        # print(self.ob)
        # print(len(self.ob))
 
    def getOdoMsg(self,  msg):
        # if self.end_pt:
        #     self.control_data['target_speed'] = 0
        #     print('goal point')
        #     pass
        # else:
        # print('get msg')
        self.start_t = time.time()
        # first_point = 0
        # if self.test_bool == False:
        self.control_data['cur_x']    = msg.pose.pose.position.x
        self.control_data['cur_y']    = msg.pose.pose.position.y
        self.control_data['cur_yaw']  = msg.twist.twist.angular.z
            # self.test_bool = True
        # ttest = msg.twist.twist.linear.x
        #print(ttest)
        
        #print('lon:',  cur_lon,  'lat:',  cur_lat, 'yaw:',  self.control_data['cur_yaw'])
        
        # self.control_data['cur_x'], self.control_data['cur_y']  = self.get_xy(cur_lat,  cur_lon,  0.9)
        # print('get odo')
        # if self.goal_x !=9999 and self.goal_y != 9999:
        if True:
            #print("boolean#######, ", self.control_data['first_check'])
            # 맨 처음 gps imu 메세지 받았을 때만 실행 됨 -> 전체 경로 받아옴.
            if self.control_data['first_check']:
                # self.goal_x, self.goal_y = 0,0
                # print('-------------------------------')
                # print('1st check')
                # print( 'cur_x:',  self.control_data['cur_x'], 'cur_y:',  self.control_data['cur_y'],  'cur_yaw:',  self.control_data['cur_yaw'])
                # print('-------------------------------')
                
                # for test
                # self.g_path_x, self.g_path_y, self.g_path_yaw ,self.g_path_k ,self.g_path_s = self.test_path()
                # print('gPathx:', self.g_path_x, '/n','gPathy:', self.g_path_y)
                
                # original 
                
                gp = GPP.Global_planner(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], self.goal_x , self.goal_y, self.goal_lane)
                gp.lane_info()
                self.g_path_x, self.g_path_y, self.g_path_yaw ,self.g_path_k ,self.g_path_s = gp.planning(gp.start_x, gp.start_y, gp.start_yaw, gp.goal_x, gp.goal_y)

                print("GPP finish")

                # print('g_path_x:',  self.g_path_x)
                self.control_data['gpp_check'] = False
                self.control_data['first_check'] = False

                # self.closest_pt(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
                # first_point = self.control_data['target_idx']
                # print('g_path_s:', self.g_path_s)

                for i in range(0, len(self.g_path_x)):
                    self.GPP_pt_msg = Point32()
                    self.GPP_pt_msg.x = self.g_path_x[i]
                    self.GPP_pt_msg.y = self.g_path_y[i]
                    self.GPP_pt_msg.z = self.g_path_yaw[i]
                    self.GPP_array_msg.points.append(self.GPP_pt_msg)
                    
                self.gpp_pub.publish(self.GPP_array_msg)    

            # d = hypot(self.control_data['cur_x']-self.g_path_x[first_point] , self.control_data['cur_y']-self.g_path_y[first_point])
            
            # if self.tick == False:
            #     plt.figure()
            #     self.tick = True

            # plt.scatter(self.g_path_x, self.g_path_y, marker='.')
            # plt.axis([-3, 2, -3,2])
            # plt.scatter(self.control_data['cur_x'], self.control_data['cur_y'], marker='x')
            
            #print("okok!2")
            #print("old set", self.old_c_d, self.old_c_d_d)


            self.control_data['LPP_mode'] = True
            delta_t = time.time() - self.start_t

            # if len(self.l_path_x) is not 0:

            
            self.s, self.c_d = self.xytosd()
            #self.s, self.c_d = 3.0
            #print("s1111: ", self.s)
            #print('-------------------------------')
            #print('2nd chek')
            print('cur_speed: ', self.control_data['speed'], 's:', self.s, 'd', self.c_d, 'c_d_d:', self.c_d_d, 'c_d_dd:', self.c_d_dd, 'delta t:', delta_t)
            
            #print('cur_x:', self.control_data['cur_x'], 'cur_y:' , self.control_data['cur_y'])  

            print( 'cur_x:',  self.control_data['cur_x'], 'cur_y:',  self.control_data['cur_y'],  'cur_yaw:',  self.control_data['cur_yaw'])
            print('cur_idx:', self.control_data['cur_idx'])
            self.ob = np.array([[-9.1428, -0.5336]])
            # print('ob x:',self.ob[:,0])
            self.control_data['look_ahead'] = self.control_data['speed']*0.5 + 1.0
            print('####look_ahead:', self.control_data['look_ahead'])
            self.l_path_x, self.l_path_y, self.l_path_k, self.l_path_s = LPP.frenet_optimal_planning(self.s, self.control_data['speed'], self.c_d, self.c_d_d , self.c_d_dd, self.ob, self.g_path_x, self.g_path_y, self.g_path_yaw, self.g_path_k, self.g_path_s, self.control_data['target_speed'])
            self.LPP_count += 1
            
            # self.control_data['cur_x'] = self.l_path_x[1]
            # self.control_data['cur_y'] = self.l_path_y[1]
            #print( 'x2:',  self.control_data['cur_x'], 'y:',  self.control_data['cur_y'],  'yaw:',  self.control_data['cur_yaw'])
            
            # self.l_path_x, self.l_path_y = 0, 0
            self.c_d_d = (self.c_d - self.old_c_d) / 100   #delta_t
            #print("before c_d_dd ", self.c_d_d, self.old_c_d_d)
            self.c_d_dd = (self.c_d_d - self.old_c_d_d) / 1000 #delta_t
            self.old_c_d = self.c_d
            self.old_c_d_d = self.c_d_d


            
            #print("hey")
            #print('-------------------------------')
            #print('3rd check')

            # print('g_path_x:', self.g_path_x)
            # print('g_path_y:', self.g_path_y)
            
            # print('l_path_x:', self.l_path_x)
            # print('l_path_y:', self.l_path_y)
            # print('l_path_s:', self.l_path_s)
            #print('-------------------------------')
            
            
            # if hypot(self.l_path_x[-1]-self.g_path_x[-1], self.l_path_y[-1]-self.g_path_y[-1]) < 1:
            #     self.control_data['target_speed'] = 0
            #     self.end_pt = True

            # plt.show()
            # self.ps(self.g_path_x, self.g_path_y, self.l_path_x, self.l_path_y)

            for i in range(len(self.l_path_x)):
                self.LPP_pt_msg = Point32()
                self.LPP_pt_msg.x = self.l_path_x[i]
                self.LPP_pt_msg.y = self.l_path_y[i]
                self.LPP_array_msg.points.append(self.LPP_pt_msg)

            self.lpp_pub.publish(self.LPP_array_msg)

            for i in range(len(self.l_path_x)):
                self.LPP_array_msg.points.pop()
            
            #LPP location
        
            #print( 'x:',  self.control_data['cur_x'], 'y:',  self.control_data['cur_y'],  'yaw:',  self.control_data['cur_yaw'])
            #print('index:',  self.control_data['target_idx'])
            # print('speed(m/s):',  self.control_data['speed'], 'speed(km/h):', self.control_data['speed']*3.6)
            if self.LPP_count > -1:
                self.control_data['steering']  = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'],  self.control_data['look_ahead'], self.control_data['LPP_mode'])
                # plt.scatter(self.g_path_x[1050:1200], self.g_path_y[1050:1200], marker='.')
                # plt.scatter(self.l_path_x, self.l_path_y, marker='o')
                # for i in range(len(self.l_path_x)):
                #     plt.text(self.l_path_x[i], self.l_path_y[i], i)
                # # for i in range(1, len(self.l_path_x)-1):
                # #     plt.text(self.l_path_x[i]+4, self.l_path_y[i]+4, self.l_path_k[i])
                # plt.scatter(self.control_data['cur_x'], self.control_data['cur_y'], marker='x')
                # plt.show()
                # plt.close()
            # self.vel_msg.pose.pose.position.x  =  self.control_data['speed']
            # self.vel_pub.publish(self.vel_msg)
            #print("okok!!3")
            # print('steering:', self.control_data['steering'])
            
            #self.control_data['steering'] = 1900            

    def run(self):
        print('controller on')
        #print(self.control_data)
        #print('controller')
        
        rospy.Subscriber("/pose", Odometry, self.getOdoMsg)
        # print('get Odo msg')
        # rospy.Subscriber('/Obs', Obs, self.getObstacleMsg)
        rospy.Subscriber('/check', Marker, self.getGoalpt)
