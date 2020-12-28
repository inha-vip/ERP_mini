#-*- coding:utf-8 -*-

import time
import serial
from nav_msgs.msg import Odometry
import rospy

class Sender:
    def __init__(self,  master):
        #self.ser = serial.Serial('/dev/ttyUSB0',115200)
        self.control_data = master.control_data
        self.vel_msg = Odometry()
        self.vel_pub = rospy.Publisher('/Vel', Odometry, queue_size=1)
        
    def calculate(self,  delta_tick, delta_time):
       
        r = 0.1 #m
        w = 2*math.pi*delta_tick/(256*12)/0.045  #0.04 is estimation
        output = r*w
        # print('from getvelocity.py  delta_tick:',delta_tick)
        # print('from getvelocity.py  rotation:', delta_tick/(256*12))
        # print('from getvelocity.py  rps:', delta_tick/(256*12)/delta_time)
        # print('from getvelocity.py  rpm:', delta_tick/(256*12)/delta_time *60)
        # print('from getvelocity.py  delta_time:', delta_time)
        print('from getvelocity.py  vel:', output * 3.6)
        self.vel_msg.pose.pose.position.x  =  output
        # self.vel_pub.publish(self.vel_msg)
        return output
        
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

    def get_clc(self, num):
        while num > 255:
            num -= 256

        digit_num = list('{0:08b}'.format(num))

        for i in range(8):
            if digit_num[i] == '0':
                digit_num[i] = '1'
            else:
                digit_num[i] = '0'

        result = ''.join(digit_num)

        return int(result,2)

        #실시간으로 계산되는 공유 데이터 값들 프로토콜애 맞게 담아서 ser.write로 쏨 
    def get_command(self):
        direction = self.control_data['direction']
        speed_Lo = self.control_data['target_speed'] & 0xFF
        speed_Hi = self.control_data['target_speed'] >> 8
        steering_Lo = self.control_data['steering'] & 0xFF
        steering_Hi = self.control_data['steering'] >> 8
        data = direction + speed_Lo + speed_Hi + steering_Lo + steering_Hi + 220 + 5 + 13 + 10  # Fixed Break values (To find the CLC)
        clc = self.get_clc(data)
        #print('steering:',  self.control_data['steering'])
        #print('True_steering:', (self.control_data['steering'] - 1550) *(30/350)  ,  'target_idx:',  self.control_data['target_idx'])
        command = [0x53, 0x54, 0x58, direction, speed_Lo, speed_Hi, steering_Lo, steering_Hi, 0xDC, 0x05, 0x00, 0x0D, 0x0A, clc]
        #print(command)
        
        return command
    

    def run(self):
        print('sender on ####################')
        
        while True:
          if self.control_data['gpp_check']:
              #print("hello")
              pass
          else:
              # print('#%$@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
              command = self.get_command()
              self.control_data['ser'].write(command) 
              #self.run_vel()
              #self.vel_pub.publish(self.vel_msg)

              #print("hi")
              time.sleep(1.0 / 100.0)
          
        print("bye sender")

