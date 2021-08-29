# -*- coding: utf-8 -*-
"""
Created on Sun Aug 29 13:22:25 2021

@author: matth

github token: 

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
"""

import socket
import time
from PCA9685 import PCA9685
from AlphaBot2 import AlphaBot2
import numpy as np
from threading import Thread



def degree_to_Pulse(angle,min_angle=-80,max_angle=80,min_pulse=50,max_pulse=1850,min_allowed_pulse=50,max_allowed_pulse=1850):
	print('angle: ',angle)
	angle=-angle ## positive angles => right or up
	pulse = (angle-min_angle)/(max_angle-min_angle)*(max_pulse-min_pulse)+min_pulse
	if pulse>max_allowed_pulse:
		pulse = max_allowed_pulse
	if pulse<min_allowed_pulse:
		pulse = int(min_allowed_pulse)
	return(pulse)



class solarschaf:
	def __init__(self):
		
		laptop_thread=Thread(target=self.laptop_communication)
		laptop_thread.start()
		
		motor_thread=Thread(target=self.motor_communication_AlphaBot2)
		motor_thread.start()
		
        
	def motor_communication_AlphaBot2(self,update_frequency = 20):	
		
		""" motor communication with Alpha Bot 2
		an analogue function should later define the communication to 
		the motor Arduino"""
		pwm=PCA9685(0x40)
		pwm.setPWMFreq(50)
		
		#Set the Horizontal servo parameters
		self.servo_h_pulse = 1050 #Sets the initial Pulse 50=> 80 grad rechts, 1050 => 0 1850=> 80 grad links
		self.servo_h_step = 0     #Sets the initial step length
		pwm.setServoPulse(0,self.servo_h_pulse)

		#Set the vertical servo parameters
		self.servo_v_pulse = 1050  #Sets the initial Pulse 50 => 85 grad oben, 1050 gerade 1500 45 grad unten
		self.servo_v_step = 0      #Sets the initial step length
		pwm.setServoPulse(1,self.servo_v_pulse)
		
		self.motor_power_left=0
		self.motor_power_right=0
        # initialize Alpha Bot 
		AlphaBot = AlphaBot2()
		AlphaBot.setMotor(self.motor_power_left, self.motor_power_right)
		
		while True:
			
			pwm.setServoPulse(0,self.servo_h_pulse)
			pwm.setServoPulse(1,self.servo_v_pulse)
			AlphaBot.setMotor(self.motor_power_left, self.motor_power_right)
			time.sleep(1/update_frequency)
        
    
	def laptop_communication(self):
		
		

		HOST = '0.0.0.0' # Server IP or Hostname
		PORT = 12352 # Pick an open Port (1000+ recommended), must match the client sport
		
		# initialize serial connection to laptop
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		print ('Socket created')
		#managing error exception
		try:
			s.bind((HOST, PORT))
		except socket.error:
			print('Bind failed ')

		s.listen(5)
		print('Socket awaiting messages')
		conn, addr = s.accept()
		print('Connected')

		# awaiting for message
		
		while True:
			data = conn.recv(1024).decode()
			print('I sent a message back in response to: ' + data)
			reply = ''

			# process your message
			if data == 'Hello':
				reply = 'Hi, back!'
			elif data == 'This is important':
				reply = 'OK, I have done the important thing you have asked me!'
				
			elif data.startswith('servo'):
				try:
					h=int(data.split(' ')[1])
					h_pulse = int(degree_to_Pulse(h))
					v=int(data.split(' ')[2])
					v_pulse = int(degree_to_Pulse(v,max_allowed_pulse=1500))
					self.servo_h_pulse=h_pulse
					self.servo_v_pulse=v_pulse
					reply = 'servo set to h_pulse='+str(h)+' v_pulse='+str(v)
					
				except:
					reply = 'servo message corrupt'

			elif data.startswith('motor'):
				try:
					left=int(data.split(' ')[1])
					right=int(data.split(' ')[2])
					print(left,right)
					
					self.motor_power_left=left
					self.motor_power_right=right
					reply = 'motor set to left '+str(left)+' right='+str(right)
					
				except:
					reply = 'servo message corrupt'




			#and so on and on until...
			elif data == 'quit':
				conn.send('Terminating')
				break
			else:
				reply = 'Unknown command'

			# Sending reply
			reply  = bytes(reply,'utf-8')
			conn.send(reply)
		
		conn.close() # Close connections
		
	
			
        
if __name__=='__main__':
	solarschaf_instance=solarschaf()
