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




class MainWindow(QMainWindow):
    def __init__(self,*args, **kwargs):
        super(MainWindow, self).__init__(*args,**kwargs)
        
        loadUi('QtMatplotlib.ui',self)
        
        
        self.mode='manual'
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
        
        
        read_thread=threading.Thread(target=self.read_u_blox)
        read_thread.start()
        
        time.sleep(0.5)
        if not self.receiver_connected:
            print('could not connect to u-blox, start simulation')
            read_thread=threading.Thread(target=self.simulate_read)
            read_thread.start()
        
        ## start plot loop. This cannot be in a seperate Thread since matplotlib can only be used in main thread
        #self.plot_x_y_with_loop()
        self.init_plot_x_y()
        
        self.update_simulation_button.clicked.connect(self.init_plot_x_y)
        self.record_point_button.clicked.connect(self.record_point)
        
        self.timer=QTimer()
        self.timer.timeout.connect(self.update_plot_x_y)
        self.timer.start(50)
        
        #self.update_simulation_button.clicked.connect(self.update_plot_x_y)
        
        
    def on_manual_mode_button_clicked(self):
        self.manual_mode_button.setStyleSheet("background-color: green")
        self.auto_mode_button.setStyleSheet("background-color: None")
        self.mode='manual'
        
    def on_auto_mode_button_clicked(self):
        
        if len(self.waypoint_x_list)>0:
            self.manual_mode_button.setStyleSheet("background-color: None")
            self.auto_mode_button.setStyleSheet("background-color: green")
            self.mode='auto'
        
        
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
                    print('lon:',long)
                    self.long_list.append(parsed_data.lon*1e-7)
                    
                    lat=parsed_data.lat*1e-7
                    print('lat:',lat)
                    self.lat_list.append(lat)
                    
                    height=parsed_data.height*1e-3
                    print('height:',height)
                    
                    print('height over mean sea level: ',parsed_data.hMSL*1e-3)
                    
                    self.x_0,self.y_0 = self.long_lat_height_to_x_y(self.long_list[0],self.lat_list[0],0)
                    x_curr,y_curr = self.long_lat_height_to_x_y(long,lat,0)
                    self.x_list.append(x_curr-self.x_0)
                    self.y_list.append(y_curr-self.y_0)
                    
                    
                    
                    
                    
                    
                    v_east = parsed_data.velE*1e-3
                    print('velocity east: ',v_east)
                    self.vx_list.append(v_east)
                    print('velocity down: ',parsed_data.velD*1e-3)
                    
                    v_north=parsed_data.velN*1e-3
                    print('velocity north: ',v_north)                    
                    self.vy_list.append(v_north)
   
    
    def long_lat_height_to_x_y(self,long,lat,height):
        
        phi = long/180*np.pi
        phi_0 = self.long_0/180*np.pi
        theta = (90-lat)/180*np.pi
        theta_0 = (90-self.lat_0)/180*np.pi
        r = 6371e3+height
        
        x= (r+height)*(phi-phi_0)*np.sin(theta)
        y= (r+height)*(theta_0-theta)
        
        return(x,y)
        
    def simulate_read(self):
        
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
        
        self.motor_speed=10
        self.steering_ratio=0.1
        self.straight_steering_ratio = 0.3
        self.motor_left=0
        self.motor_right=0
        
        ## random walk
        while not keyboard.is_pressed('q'):
            
            if self.mode=='manual':
                print('manual mode')
                ## check keyboard and send messages accordingly
                
                    
                if keyboard.is_pressed('up'):
                    self.motor_left=self.motor_speed
                    self.motor_right=self.motor_speed
                    
                elif keyboard.is_pressed('down'):
                    self.motor_left=-self.motor_speed
                    self.motor_right=-self.motor_speed
                else:
                    self.motor_left=0
                    self.motor_right=0
                    
                if keyboard.is_pressed('left'):
                    self.motor_left-=self.motor_speed*self.steering_ratio
                    self.motor_right+=self.motor_speed*self.steering_ratio
                if keyboard.is_pressed('right'):
                    self.motor_left+=self.motor_speed*self.steering_ratio
                    self.motor_right-=self.motor_speed*self.steering_ratio
                
                
                
                
            if self.mode=='auto':
                print('auto mode')
                self.calculate_auto()
                
            
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
            if self.sim_phi>np.pi:
                self.sim_phi-=2*np.pi
                
            if self.sim_phi<-np.pi:
                self.sim_phi+=2*np.pi
            
            print('sim_phi',self.sim_phi)
            self.x_list.append(self.sim_x)            
            self.y_list.append(self.sim_y)
            
            self.long_list.append(self.sim_x)
            self.lat_list.append(self.sim_y)
            
            self.vx_list.append(self.sim_vx)
            self.vy_list.append(self.sim_vy)
            
          #  print('added lat, long ',self.y_list[-1],self.x_list[-1])
          #  print('added vx, vy ',self.vx_list[-1],self.vy_list[-1])
            time.sleep(dt)
            
    def calculate_auto(self,delta_phi_turn=90/180*np.pi,d_min=0.1):
        self.d_min=d_min
        
        if self.next_waypoint_index >= len(self.waypoint_y_list):
            self.next_waypoint_index =0
        
        
        ## calculate relative deviation        
        
        self.delta_x=self.waypoint_x_list[self.next_waypoint_index]-self.x_list[-1]
        self.delta_y=self.waypoint_y_list[self.next_waypoint_index]-self.y_list[-1]
        
        self.delta_abs = np.sqrt(self.delta_y**2+self.delta_x**2)
        
        ## calculate angle
        self.target_phi = np.arctan(self.delta_y/self.delta_x)
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
            
            self.motor_right=self.motor_speed*(1+self.delta_phi*self.straight_steering_ratio)
            self.motor_left=self.motor_speed*(1-self.delta_phi*self.straight_steering_ratio)
            
        if self.delta_abs<self.d_min:
            self.next_waypoint_index+=1
    
        
        
        
    def init_plot_x_y(self,blit=True):
        self.blit=blit
        
        
        
        if len(self.y_list)<2:
            self.points, = self.plot_widget.canvas.axes.plot([0,1], [0,1], 'ro')
            self.line, = self.plot_widget.canvas.axes.plot([0,1],[0,1],linewidth=1,color='lightblue')
            
            self.plot_widget.canvas.axes.set_xlim(-5, 5)
            self.plot_widget.canvas.axes.set_ylim(-5, 5)
        
        else:
            self.points, = self.plot_widget.canvas.axes.plot( self.x_list[-100::],self.y_list[-100::], 'ro')
            self.line, = self.plot_widget.canvas.axes.plot( self.x_list[-100::],self.y_list[-100::],linewidth=1,color='lightblue')
#            self.plot_widget.canvas.axes.relim()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
            
            xrange=max(self.x_list)-min(self.x_list)
            yrange=max(self.y_list)-min(self.y_list)
            self.plot_widget.canvas.axes.set_xlim(min(self.x_list)-0.1*xrange,max(self.x_list)+0.1*xrange)
            self.plot_widget.canvas.axes.set_ylim(min(self.y_list)-0.1*yrange,max(self.y_list)+0.1*yrange)
            self.plot_widget.canvas.draw()
        self.plot_widget.canvas.axes.set_xlabel(r'$x$ (m, Richtung Osten)')
        self.plot_widget.canvas.axes.set_ylabel(r'$y$ (m, Richtung Norden)')
        
        
        self.arrow=self.plot_widget.canvas.axes.annotate('',xy=(0,1),xytext=(0,0),arrowprops={'arrowstyle':'->'})
        if blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
      
        
    def update_plot_x_y(self):
        
        if len(self.y_list)<1000:
            self.points.set_data(self.x_list,self.y_list)
            self.line.set_data(self.x_list,self.y_list)
            
        else:
            self.points.set_data(self.x_list[-1000::],self.y_list[-1000::])
            self.line.set_data(self.x_list[-1000::],self.y_list[-1000::])
        if len(self.y_list)>2:
           # self.plot_widget.canvas.axes.set_xlim(min(self.y_list)-1,max(self.y_list)+1)
           # self.plot_widget.canvas.axes.set_ylim(min(self.x_list)-1,max(self.x_list)+1)
            self.arrow.set_position((self.x_list[-1],self.y_list[-1]))
            self.arrow.xy=(self.x_list[-1]+self.vx_list[-1],self.y_list[-1]+self.vy_list[-1])
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
        
        
        self.waypoints, = self.plot_widget.canvas.axes.plot( self.waypoint_x_list,self.waypoint_y_list,'ro',markersize=10,color='blue')
        self.plot_widget.canvas.axes.draw_artist(self.waypoints)
        
        if self.blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
        
        
        
        
            
        
      
            
            

if __name__=='__main__':
    
    
    app=QApplication(sys.argv)
    main = MainWindow()
    main.show()
    
    
    sys.exit(app.exec_())
    