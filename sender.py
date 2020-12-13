#-*- coding:utf-8 -*-

import time
import serial

class Sender:
    def __init__(self,  master):
        #self.ser = serial.Serial('/dev/ttyUSB0',115200)
        self.control_data = master.control_data
        
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
        while True:
            if self.control_data['gpp_check']:
                pass
            else:
                command = self.get_command()
                #print(command)
                # line = []
                # for c in self.control_data['ser'].read():
                #     line.append(int(c))
                #     print(len(line))
                #     if c == 10:
                #         print(line)
                #         del line[:]



                # sl = self.control_data['ser'].readline()
                # print(sl)

                self.control_data['ser'].write(command) 

                time.sleep(1.0 / 100.0)
