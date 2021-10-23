# -*- coding: utf-8 -*-
"""
Created on Tue Aug 24 20:48:15 2021

@author: matth
"""


import sys
import numpy as np
from serial import Serial
from pyubx2 import UBXReader
import keyboard
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import os

from PyQt5.QtWidgets import QWidget,QVBoxLayout,QMainWindow,QApplication
from PyQt5.uic import loadUi
from PyQt5.QtCore import QTimer

    
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

import BNO055



class MainWindow(QMainWindow):
    def __init__(self,*args, **kwargs):
        super(MainWindow, self).__init__(*args,**kwargs)
        
        loadUi('QtMatplotlib.ui',self)
        
        self.left_button_clicked=False
        self.right_button_clicked=False
        self.forward_button_clicked=False
        self.backward_button_clicked=False
        
        self.left_button.pressed.connect(self.left_clicked)
        self.right_button.pressed.connect(self.right_clicked)
        self.forward_button.pressed.connect(self.forward_clicked)
        self.backward_button.pressed.connect(self.backward_clicked)
        
        self.left_button.released.connect(self.left_released)
        self.right_button.released.connect(self.right_released)
        self.forward_button.released.connect(self.forward_released)
        self.backward_button.released.connect(self.backward_released)
        
        
        self.mode='manual'
        self.simulate=False
        self.receiver_connected=False
        self.next_waypoint_index=0
        
        self.lat_list=[]
        self.long_list=[]
        
        self.long_0=11.011380329757094
        self.lat_0=49.5856380908053
        
        self.x_list=[]
        self.y_list=[]
        
        self.waypoint_long_list=[]
        self.waypoint_lat_list=[]
        
        self.waypoint_y_list=[]
        self.waypoint_x_list=[]
        
        self.vx_list=[]
        self.vy_list=[]
        self.phi_list=[]
        self.alpha=0
        self.beta=0
        self.gamma=0
        
        self.motor_left=0
        self.motor_right=0
        
        
        
        read_thread=threading.Thread(target=self.read_u_blox)
        read_thread.start()
        
        
        
        time.sleep(0.5)
        if not self.receiver_connected:
            print('could not connect to u-blox, start simulation')
        main_control_thread=threading.Thread(target=self.main_control)
        main_control_thread.start()
            
        ## start reading the orientation sensor
        self.init_euler()
        self.angle_timer=QTimer()
        self.angle_timer.timeout.connect(self.read_euler)
        self.angle_timer.start(100)
        
        ## start reading the ultrasonic sensors
        
        
        ## start plot loop. This cannot be in a seperate Thread since matplotlib can only be used in main thread
        #self.plot_x_y_with_loop()
        self.init_plot_x_y()
        
        self.update_simulation_button.clicked.connect(self.init_plot_x_y)
        self.record_point_button.clicked.connect(self.record_point)
        self.load_curve_button.clicked.connect(self.load_curve)
        
        self.timer=QTimer()
        self.timer.timeout.connect(self.update_plot_x_y)
        self.timer.start(50)
        
        
        
        #self.update_simulation_button.clicked.connect(self.update_plot_x_y)
    
    def left_clicked(self):
        self.left_button_clicked=True
        
    def right_clicked(self):
        self.right_button_clicked=True
        
    def forward_clicked(self):
        self.forward_button_clicked=True
        
    def backward_clicked(self):
        self.backward_button_clicked=True
        
        
    def left_released(self):
        self.left_button_clicked=False
        
    def right_released(self):
        self.right_button_clicked=False
        
    def forward_released(self):
        self.forward_button_clicked=False
        
    def backward_released(self):
        self.backward_button_clicked=False
        
        
        
    def on_manual_mode_button_clicked(self):
        self.manual_mode_button.setStyleSheet("background-color: green")
        self.auto_mode_button.setStyleSheet("background-color: None")
        self.drive_to_click_button.setStyleSheet("background-color: None")
        self.mode='manual'
        
    def on_auto_mode_button_clicked(self):
        
        if len(self.waypoint_x_list)>0:
            self.manual_mode_button.setStyleSheet("background-color: None")
            self.auto_mode_button.setStyleSheet("background-color: green")
            self.drive_to_click_button.setStyleSheet("background-color: None")
            self.mode='auto'
            
    def on_drive_to_click_button_clicked(self):
        
        self.manual_mode_button.setStyleSheet("background-color: None")
        self.auto_mode_button.setStyleSheet("background-color: None")
        self.drive_to_click_button.setStyleSheet("background-color: green")
        self.mode='drive_to_clicked'
        
        self.motor_left=0
        self.motor_right=0
        
        if len(self.x_list)>0:
            self.target_x = self.x_list[-1]
            self.target_y = self.y_list[-1]
            
        else:
            self.target_x=0.1
            self.target_y=0.1
        
        self.plot_widget.canvas.callbacks.connect('button_press_event', self.set_target_on_click)
        
    def read_u_blox(self):
        
        if os.name=='nt':      ## check for windows
            print('system is nt, probably laptop')
            try:
                stream = Serial('COM4', 9600, timeout=3)
                self.receiver_connected=True
            except:
                pass
        elif os.name=='posix':
            try:
                stream = Serial('/dev/ttyACM0', 9600, timeout=3)   ## check for Linux
                print('system is posix, probably laptop')
                self.receiver_connected=True
            except:
                pass
            
        if self.receiver_connected:
            ubr = UBXReader(stream)
            
            print('connection to UBX established')
    
            while not keyboard.is_pressed('q'):
                (raw_data, parsed_data) = ubr.read()
                #print(parsed_data.identity)
                if parsed_data.identity == "NAV-PVT":
                    #print(parsed_data)
                    
                    long=parsed_data.lon*1e-7
                    #print('lon:',long)
                    self.long_list.append(parsed_data.lon*1e-7)
                    
                    lat=parsed_data.lat*1e-7
                    #print('lat:',lat)
                    self.lat_list.append(lat)
                    
                    height=parsed_data.height*1e-3
                    #print('height:',height)
                    
                    #print('height over mean sea level: ',parsed_data.hMSL*1e-3)
                    
                    self.x_0,self.y_0 = self.long_lat_height_to_x_y(self.long_list[0],self.lat_list[0],0)
                    x_curr,y_curr = self.long_lat_height_to_x_y(long,lat,0)
                    self.x_list.append(x_curr-self.x_0)
                    self.y_list.append(y_curr-self.y_0)
                    
                    
                    
                    
                    
                    
                    v_east = parsed_data.velE*1e-3
                    #print('velocity east: ',v_east)
                    self.vx_list.append(v_east)
                    #print('velocity down: ',parsed_data.velD*1e-3)
                    
                    v_north=parsed_data.velN*1e-3
                    #print('velocity north: ',v_north)                    
                    self.vy_list.append(v_north)
                    
                    
                    phi=np.pi/180*(90-self.alpha)
                    if phi>np.pi:
                        phi-=2*np.pi
                
                    if phi<-np.pi:
                        phi+=2*np.pi
                    self.phi_list.append(phi)
                    
    def init_motor(self):
        
        baud = 115200
        port='/dev/ttyUSB0'
        self.motor_serial =serial.Serial(port,baud,timeout=0.005)
        print(self.motor_serial.name,'motor connection established')
        
    
    def set_motor(self,left,right):
            command = 'l'+str(int(set_left))+'r'+str(int(set_right))+'\n'
            self.motor_serial.write(bytes(command,'utf-8'))
            reply = s.read(1000)
            print (reply)
        
                    
    def init_euler(self):
        self.bno=BNO055.BNO055()
        #bno.reset()
        if self.bno.begin() is not True:
            print("Error initializing device")
            exit()
        time.sleep(1)
        self.bno.setExternalCrystalUse(False)
        print('initializing euler finished')
        
        self.alpha=0
        self.beta=0
        self.gamma=0
        self.euler_available=True
        
        
    def read_euler(self):
        
        self.alpha, self.beta,self.gamma = self.bno.getVector(BNO055.BNO055.VECTOR_EULER)
        #print(self.alpha,self.beta,self.gamma)
            
    def long_lat_height_to_x_y(self,long,lat,height):
        
        phi = long/180*np.pi
        phi_0 = self.long_0/180*np.pi
        theta = (90-lat)/180*np.pi
        theta_0 = (90-self.lat_0)/180*np.pi
        r = 6371e3+height
        
        x= (r+height)*(phi-phi_0)*np.sin(theta)
        y= (r+height)*(theta_0-theta)
        
        return(x,y)
        
        
    def main_control(self):
        
        dt=0.1
        self.sim_v=0
        self.sim_omega = 0
        self.sim_x=0
        self.sim_y=0
        self.sim_phi = 0
        self.sim_tau_v = 0.2
        self.sim_tau_omega=0.2
        self.sim_mass=100
        self.sim_m_o_inertia = 10
        
        self.stdev=1 ### standard deviation of measurement values in m 
        
        if self.simulate==True:
            self.motor_speed=10
            self.steering_ratio=0.1
            self.straight_steering_ratio = 0.3
            
            
        else:
            self.motor_speed=20
            self.steering_ratio=1
            self.straight_steering_ratio = 0.3
        
        ## random walk
        while not keyboard.is_pressed('q'):
            
            if self.mode=='manual':
                #print('manual mode')
                ## check keyboard and send messages accordingly
                
                    
                    
                if self.forward_button_clicked :
                    self.motor_left+=1
                    self.motor_right+=1
                    
                elif self.backward_button_clicked :
                    
                    self.motor_left -=1
                    self.motor_right -=1
                
                    
                elif self.left_button_clicked :
                    self.motor_left-=2*self.steering_ratio
                    self.motor_right+=1*self.steering_ratio
                    
                elif self.right_button_clicked :
                    self.motor_left+=1*self.steering_ratio
                    self.motor_right-=2*self.steering_ratio
                    
                else: 
                    self.motor_left=0.5*self.motor_left
                    self.motor_right=0.5*self.motor_right
                
                
                
                
            if self.mode=='auto':
                print('auto mode')
                self.calculate_auto()
                
            if self.mode=='drive_to_clicked':
                print('drive to point: klick in plot')
                
                self.calculate_drive_to_point(self.target_x,self.target_y,d_min=0.1)
                
            if not self.simulate:
                print('set motor to l/r ',self.motor_left,self.motor_right)
                #self.set_motor(motor_left,motor_right)
                
            if self.simulate:
                self.sim_v += (self.motor_left+self.motor_right)/self.sim_mass 
                self.sim_v -= self.sim_v*dt/self.sim_tau_v
                self.sim_omega += (self.motor_right-self.motor_left)/self.sim_m_o_inertia
                self.sim_omega -= self.sim_omega*dt/self.sim_tau_omega
            
            
                print('sim_omega:',self.sim_omega)
            
                self.sim_vx=self.sim_v*np.cos(self.sim_phi)
                self.sim_vy=self.sim_v*np.sin(self.sim_phi)
            
            
                self.sim_x+=self.sim_vx
                self.sim_y+=self.sim_vy
                self.sim_phi +=self.sim_omega
            else:
                self.sim_phi=np.pi/180*(90-self.alpha)
                  
            
            if self.sim_phi>np.pi:
                self.sim_phi-=2*np.pi
                
            if self.sim_phi<-np.pi:
                self.sim_phi+=2*np.pi
            
            if self.simulate:
            
                print('sim_phi',self.sim_phi)
                self.x_list.append(self.sim_x)            
                self.y_list.append(self.sim_y)
                
                self.long_list.append(self.sim_x)
                self.lat_list.append(self.sim_y)
                
                self.vx_list.append(self.sim_vx)
                self.vy_list.append(self.sim_vy)
                self.phi_list.append(self.sim_phi)
                
            
            time.sleep(dt)
          
            
    def calculate_drive_to_point(self,waypoint_x,waypoint_y,delta_phi_turn=10/180*np.pi,d_min=0.1):
        ## calculate relative deviation        
        
        print('bin in calculate_drive_to_point',waypoint_x,waypoint_y)
        self.d_min=d_min
        
        self.delta_x=waypoint_x-self.x_list[-1]
        self.delta_y=waypoint_y-self.y_list[-1]
        
        self.delta_abs = np.sqrt(self.delta_y**2+self.delta_x**2)
        
        ## calculate angle
        
        if not self.delta_x==0:
            self.target_phi = np.arctan(self.delta_y/self.delta_x)
        else:
            self.target_phi = np.pi
        if self.delta_x<0 and self.delta_y>0:
            self.target_phi+=np.pi
            
        if self.delta_x<0 and self.delta_y<0:
            self.target_phi-=np.pi
         
        self.delta_phi=self.target_phi-self.sim_phi
        
        if self.delta_phi>np.pi:
            self.delta_phi-=2*np.pi
        if self.delta_phi<-np.pi:
            self.delta_phi+=2*np.pi
        
        
        
        
       # print('delta_y',self.delta_y,'delta_x',self.delta_x)
        print('next_waypoint_index',self.next_waypoint_index)
        print('phi',self.sim_phi,'target_phi',self.target_phi,'delta_phi',self.delta_phi)
        
        
        ## if not already at arget point:
        
        if self.delta_abs>d_min:
            
            ## if angle is too high, just turn
            
            if self.delta_phi <-delta_phi_turn:
                ## turn right
                print('right')
                self.motor_right=-self.steering_ratio*(self.motor_speed+0.1*self.motor_speed)
                self.motor_left=+self.steering_ratio*(self.motor_speed+0.1*self.motor_speed)
    
            elif self.delta_phi > delta_phi_turn:   
                
                ## turn left
                print('left')
                self.motor_right=self.steering_ratio*(self.motor_speed+0.1*self.motor_speed)
                self.motor_left=-self.steering_ratio*(self.motor_speed+0.1*self.motor_speed)
                
            else:
                ## go forward
                
                print('forward')
                
                stop_factor=min([1,self.delta_abs/5/d_min])
                
                print(stop_factor)
                
                self.motor_right=stop_factor*self.motor_speed*(1+self.delta_phi*self.straight_steering_ratio)
                self.motor_left=stop_factor*self.motor_speed*(1-self.delta_phi*self.straight_steering_ratio)
                
                
        else:
            self.motor_right=0
            self.motor_left=0
            
    def calculate_auto(self):
        
        
        if self.next_waypoint_index >= len(self.waypoint_y_list):
            self.next_waypoint_index =0
        ## calculate motor PWM
        
        self.calculate_drive_to_point(self.waypoint_x_list[self.next_waypoint_index],self.waypoint_y_list[self.next_waypoint_index])
        
        
            
        if self.delta_abs<self.d_min:
            self.next_waypoint_index+=1
    
        
        
        
    def init_plot_x_y(self,blit=True):
        self.blit=blit
        self.plot_memory=100
        
        
        if len(self.y_list)<2:
            self.points, = self.plot_widget.canvas.axes.plot([0,1], [0,1], 'ro',markersize=2)
            self.line, = self.plot_widget.canvas.axes.plot([0,1],[0,1],linewidth=1,color='lightblue')
            
            self.plot_widget.canvas.axes.set_xlim(-5, 5)
            self.plot_widget.canvas.axes.set_ylim(-5, 5)
        
        else:
            self.points, = self.plot_widget.canvas.axes.plot( self.x_list[-self.plot_memory::],self.y_list[-self.plot_memory::], 'ro',markersize=2)
            self.line, = self.plot_widget.canvas.axes.plot( self.x_list[-self.plot_memory::],self.y_list[-self.plot_memory::],linewidth=1,color='lightblue')
#            self.plot_widget.canvas.axes.relim()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
            
            
            
            #xrange=max(self.x_list)-min(self.x_list)
            #yrange=max(self.y_list)-min(self.y_list)
            #self.plot_widget.canvas.axes.set_xlim(min(self.x_list)-0.1*xrange,max(self.x_list)+0.1*xrange)
            #self.plot_widget.canvas.axes.set_ylim(min(self.y_list)-0.1*yrange,max(self.y_list)+0.1*yrange)
            
            self.plot_widget.canvas.axes.autoscale()
            self.plot_widget.canvas.axes.autoscale()
            
            self.plot_widget.canvas.draw()
        self.plot_widget.canvas.axes.set_xlabel(r'$x$ (m, Richtung Osten)')
        self.plot_widget.canvas.axes.set_ylabel(r'$y$ (m, Richtung Norden)')
        
        ## connect click event to function
        
        
        self.arrow=self.plot_widget.canvas.axes.annotate('',xy=(0,1),xytext=(0,0),arrowprops={'arrowstyle':'->'})
        if blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
      
        
    def update_plot_x_y(self):
        
        if len(self.y_list)<self.plot_memory:
            self.points.set_data(self.x_list,self.y_list)
            self.line.set_data(self.x_list,self.y_list)
            
        else:
            self.points.set_data(self.x_list[-self.plot_memory::],self.y_list[-self.plot_memory::])
            self.line.set_data(self.x_list[-self.plot_memory::],self.y_list[-self.plot_memory::])
        if len(self.y_list)>2:
           # self.plot_widget.canvas.axes.set_xlim(min(self.y_list)-1,max(self.y_list)+1)
           # self.plot_widget.canvas.axes.set_ylim(min(self.x_list)-1,max(self.x_list)+1)
            self.arrow.set_position((self.x_list[-1],self.y_list[-1]))
            
            phi=self.phi_list[-1]
            x_norm=0.1*np.cos(phi)
            y_norm=0.1*np.sin(phi)
            
            print('x_norm,y_norm',x_norm,y_norm)
            self.arrow.xy=(self.x_list[-1]+x_norm,self.y_list[-1]+y_norm)
            
            self.plot_widget.fig.canvas.draw()
            
            if self.blit:
                # restore background
                self.plot_widget.canvas.restore_region(self.axbackground)
    
                # redraw just the points
                
                self.plot_widget.canvas.axes.draw_artist(self.points)
                self.plot_widget.canvas.axes.draw_artist(self.line)
                self.plot_widget.canvas.axes.draw_artist(self.arrow)
    
                # fill in the axes rectangle
                self.plot_widget.canvas.blit(self.plot_widget.canvas.axes.bbox)
            
            else:
            # redraw everything
                self.plot_widget.canvas.draw()
             
    def record_point(self):
        
        print('point recorded')
        
        
        self.waypoint_x_list.append(self.x_list[-1])
        self.waypoint_y_list.append(self.y_list[-1])
        
        self.waypoint_long_list.append(self.long_list[-1])
        self.waypoint_lat_list.append(self.lat_list[-1])
        
        
        self.waypoints, = self.plot_widget.canvas.axes.plot( self.waypoint_x_list,self.waypoint_y_list,'ro',markersize=2,color='blue')
        self.plot_widget.canvas.axes.draw_artist(self.waypoints)
        
        if self.blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
        
        
    def load_curve(self):
        
        scale_factor=0.01
        data = np.loadtxt('waypoints/Solarschaf_Detailliert.csv',delimiter=',',skiprows=1).transpose()*scale_factor
        print(data)
    
        self.waypoint_x_list = data[0]
        self.waypoint_y_list = -data[1]
        
        ## shift to 0 
        
        self.waypoint_x_list-=self.waypoint_x_list[0]
        self.waypoint_y_list-=self.waypoint_y_list[0]
        
        
        self.waypoints, = self.plot_widget.canvas.axes.plot( self.waypoint_x_list,self.waypoint_y_list,'ro',markersize=2,color='blue')
        self.plot_widget.canvas.axes.draw_artist(self.waypoints)
        
        if self.blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
        
        
        
    def set_target_on_click(self,event):
        print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              ('double' if event.dblclick else 'single', event.button,
               event.x, event.y, event.xdata, event.ydata))
        
        self.target_x=event.xdata
        self.target_y=event.ydata
        line, = self.plot_widget.canvas.axes.plot([event.xdata,event.xdata],[event.ydata,event.ydata],'ro',color='green')
        self.plot_widget.canvas.axes.draw_artist(line)
        if self.blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
        
            
        
      
            
            

if __name__=='__main__':
    
    
    app=QApplication(sys.argv)
    main = MainWindow()
    main.show()
    
    
    sys.exit(app.exec_())
    
