# -*- coding: utf-8 -*-
"""
Created on Fri Oct  1 20:21:44 2021

@author: matth
"""

import serial 
import time
import keyboard


baud = 115200
port='/dev/ttyUSB0'

dt = 0.01

set_left=0
set_right=0

motor_speed=40
steering_ratio=1

if __name__=='__main__':
    s =serial.Serial(port,baud,timeout=0.005)
    print(s.name)
    
    def ardu_msg(left,right):
            command = 'l'+str(int(set_left))+'r'+str(int(set_right))+'\n'
            s.write(bytes(command,'utf-8'))
            #reply = s.read(1000)
            #print (reply)
    
    
    last_t = time.time()
    
    while True:
        ## check keyboard and send messages accordingly
        if time.time()-last_t >dt:
            
            if keyboard.is_pressed('q'):
                break
            
            if keyboard.is_pressed('w') and motor_speed<=40: 
                motor_speed+=1
                print('motor_speed = ',motor_speed)
            
            if keyboard.is_pressed('s') and motor_speed>0:
                motor_speed-=1
                print('motor_speed = ',motor_speed)
                
            if keyboard.is_pressed('up'):
                motor_left=motor_speed
                motor_right=motor_speed
                
            elif keyboard.is_pressed('down'):
                
                motor_left=-motor_speed
                motor_right=-motor_speed
            else:
                motor_left=0
                motor_right=0
                
            if keyboard.is_pressed('left'):
                motor_left-=motor_speed*steering_ratio
                motor_right+=motor_speed*steering_ratio
            if keyboard.is_pressed('right'):
                motor_left+=motor_speed*steering_ratio
                motor_right-=motor_speed*steering_ratio
            
            
            set_left=motor_left
            set_right=motor_right
            ardu_msg(motor_left,motor_right)
            last_t=time.time()
        
        if s.in_waiting:
            print(s.read(100).decode('utf-8'))
        
