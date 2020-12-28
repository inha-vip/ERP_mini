#!/usr/local/bin/python
# -*- coding: utf-8 -*-

import math
import time
from nav_msgs.msg import Odometry
import rospy


class GetVelocity:
    def __init__(self,  master):
        self.control_data = master.control_data
        self.vel_msg = Odometry()
        self.vel_pub = rospy.Publisher('/Vel', Odometry, queue_size=1)
        # rate = rospy.Rate(50) 

    def calculate(self,  delta_tick, delta_time):
        # n = data
        # N= 152
        # t = 1
        
        # w = 2*math.pi*n/N/t
        
        # r = 0.1 #m
        # v = r*w*3.6 #km/h
        # output = v
        
        r = 0.1 #m
        #if delta_tick < 0:
        #    delta_tick += (256*256-1)
        #    print("delta < 0 : ", delta_tick)
        w = 2*math.pi*delta_tick/(256*12)/0.045  #0.04 is estimation
        output = r*w
        # print('from getvelocity.py  delta_tick:',delta_tick)
        # print('from getvelocity.py  rotation:', delta_tick/(256*12))
        # print('from getvelocity.py  rps:', delta_tick/(256*12)/delta_time)
        # print('from getvelocity.py  rpm:', delta_tick/(256*12)/delta_time *60)
        # print('from getvelocity.py  delta_time:', delta_time)
        # print('from getvelocity.py  vel:', output * 3.6)
        self.vel_msg.pose.pose.position.x  =  output
        self.vel_pub.publish(self.vel_msg)
        return output

    
    # def run_vel(self):
    #     # 쓰레드 종료될때까지 계속 돌림
    #     while not self.control_data['exitThread']:
    #         #데이터가 있있다면
    #         for c in self.control_data['ser'].read():
    #             #line 변수에 차곡차곡 추가하여 넣는다.
    #             self.control_data['line'].append(int(c))

    #             if c == 10: #라인의 끝을 만나면..
    #                 #데이터 처리 함수로 호출

    #                 mini = self.control_data['line'][6:9]
    #                 #speed = mini[1] * 256 + mini[0]


    #                 speed = mini[-2:-1] + mini[-3:-2]
    #                 #print('mini speed: ', speed)
    #                 if speed : 
    #                     if speed[0] < 5 :
    #                       temp = speed[0] * 256 + speed[1]
    #                       self.control_data['speed'] = self.calculate(temp)
                            
    #                 #line 변수 초기화
    #                 del self.control_data['line'][:] 



    def run_vel(self):
        # 쓰레드 종료될때까지 계속 돌림
        print('get velocity on')
        old_tick = 0
        tmp_tick = 0
        last_delta = 0
        s_delta = 0
        while not self.control_data['exitThread']:
            #데이터가 있있다면
            start_time = time.time()
            for c in self.control_data['ser'].read():
                #line 변수에 차곡차곡 추가하여 넣는다.
                self.control_data['line'].append(int(c))

                if c == 10: #라인의 끝을 만나면..
                    #데이터 처리 함수로 호출
                    
                  mini = self.control_data['line'][11:15]
                    #tick = mini[1] * 256 + mini[0]
                  #print('mini:', mini) 
                  tick = mini[-2:-1] + mini[-3:-2]
                  # print('mini tick: ', tick)
                  # print('tick[0]', tick[0])
                  #if self.control_data['line'][1] is 83:
                  if len(tick) >= 2: 
                      if tick[0] < 256 :
                        tmp_tick = tick[0] * 256 + tick[1]
                      #   print('sum tick:', tmp_tick)
                        delta_time = time.time() - start_time
                        # print('dt:',  delta_time)
                      #   start_time = time.time()                       
                        delta_tick = tmp_tick - old_tick
                        if delta_tick < 0:
                          s_delta = delta_tick + 256*256
                        else:
                          s_delta = delta_tick  
                        if(abs(delta_tick) > 500):
                          # print("error")
                          delta_tick = last_delta
                        else:
                          last_delta = s_delta
                        #print(delta_tick)
                        #print(self.control_data['line'])
                        # print("temp v : ", tmp_tick, " : ", old_tick)
                        # print("original : ", delta_tick)
                        #print(self.control_data['line'])
                      #   print('d tic:', delta_tick)
                        old_tick = tmp_tick
                        self.control_data['speed'] = self.calculate(delta_tick,delta_time)
                  #line 변수 초기화
                  del self.control_data['line'][:] 

    # def run_vel(self):
    #     # 쓰레드 종료될때까지 계속 돌림
    #     while not self.control_data['exitThread']:
    #         #데이터가 있있다면
    #         # sl = self.control_data['ser'].readline()
    #         # idd = 0
    #         # for idd in range(len(sl)): 
    #         #     print('##number',idd ,':', sl[idd]) 

    #         for c in self.control_data['ser'].read():
    #             #line 변수에 차곡차곡 추가하여 넣는다.
    #             self.control_data['line'].append(int(c))
                
    #             if c == 10: #라인의 끝을 만나면..
    #                 #데이터 처리 함수로 호출

    #                 mini = self.control_data['line'][6:15]
    #                 print('mini:', mini)
    #                 #speed = mini[1] * 256 + mini[0]


    #                 speed = mini[-2:-1] + mini[-3:-2]
    #                 #print('mini speed: ', speed)
    #                 if speed : 
    #                     if speed[0] < 5 :
    #                       temp = speed[0] * 256 + speed[1]
    #                       self.control_data['speed'] = self.calculate(temp)
                            
    #                 #line 변수 초기화
    #                 del self.control_data['line'][:]  
    # def run_vel(self):
    #     # 쓰레드 종료될때까지 계속 돌림
    #     while not self.control_data['exitThread']:
    #         #데이터가 있있다면
    #         sl = self.control_data['ser'].readline()
            
    #         idd = 0
    #         for idd in range(len(sl)): 
    #             print('##number',idd ,':', sl[idd]) 

    #         time.sleep(3)   

    #         for c in self.control_data['ser'].read():
    #             #line 변수에 차곡차곡 추가하여 넣는다.
    #             self.control_data['line'].append(int(c))
    #             #print(hex(c))
    #             #time.sleep(5)

    #             if c == 10: #라인의 끝을 만나면..
    #                 #데이터 처리 함수로 호출

    #                 mini = self.control_data['line'][6:9]
    #                 #speed = mini[1] * 256 + mini[0]


    #                 speed = mini[-2:-1] + mini[-3:-2]
    #                 #print('mini speed: ', speed)
    #                 if speed : 
    #                     if speed[0] < 5 :
    #                       temp = speed[0] * 256 + speed[1]
    #                       self.control_data['speed'] = self.calculate(temp)
                            
    #                 #line 변수 초기화
    #                 del self.control_data['line'][:]

    # def run_vel(self):
    #     # 쓰레드 종료될때까지 계속 돌림
    #     while not self.control_data['exitThread']:
    #         #데이터가 있있다면
    #         for c in self.control_data['ser'].read():

    #             self.control_data['speed'] = c

                #line 변수에 차곡차곡 추가하여 넣는다.
                #self.control_data['line'].append(int(c))

                #if c == 10: #라인의 끝을 만나면..
                #     #데이터 처리 함수로 호출

                 #   mini = self.control_data['line']
                #     #speed = mini[1] * 256 + mini[0]

                #     speed = mini[-2:-1] + mini[-3:-2]
                #     if speed : 
                #         if speed[0] < 5 :
                #             temp = speed[0] * 256 + speed[1]
                            
                #     #line 변수 초기화
                 #   del self.control_data['line'][:]     

