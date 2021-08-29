# -*- coding: utf-8 -*-
"""
Created on Sat Apr 11 17:15:06 2020

@author: matth

this script should run on the computer which acts as remote control
and goes together with "Laptop_Fernsteuerung.py" on the raspberry pi 
(start with python3)
"""

import socket
import keyboard
import time 

HOST = '192.168.0.96' # Enter IP or Hostname of your server
PORT = 12352 # Pick an open Port (1000+ recommended), must match the server port



servo_h=0
servo_v=0

motor_left=0
motor_right=0

motor_speed=20
steering_ratio=1

dt=0.02 ##update time in us 

if __name__=="__main__":
    
    ## connect
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    
    def set_motor(left,right):
            command = 'motor '+str(int(motor_left))+' '+str(int(motor_right))
            s.send(bytes(command,'utf-8'))
            reply = s.recv(1024)
            print (reply)
    
    def set_servo(h,v):
            command = 'servo '+str(int(servo_h))+' '+str(int(servo_v))
            s.send(bytes(command,'utf-8'))
            reply = s.recv(1024)
            print (reply)
    
    last_t = time.time()
    
    while True:
        ## check keyboard and send messages accordingly
        if time.time()-last_t >dt:
            if keyboard.is_pressed('w'):
                servo_v+=1
            if keyboard.is_pressed('s'):
                servo_v-=1
                
            if keyboard.is_pressed('a'):
                servo_h-=1
            if keyboard.is_pressed('d'):
                servo_h+=1
                
                
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
            
            set_motor(motor_left,motor_right)
            set_servo(servo_h,servo_v)
            last_t=time.time()
        
            
            