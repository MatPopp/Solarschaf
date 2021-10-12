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

from PyQt5.QtWidgets import QWidget,QVBoxLayout,QMainWindow,QApplication
from PyQt5.uic import loadUi
from PyQt5.QtCore import QTimer

    
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)


class MainWindow(QMainWindow):
    def __init__(self,*args, **kwargs):
        super(MainWindow, self).__init__(*args,**kwargs)
        
        loadUi('QtMatplotlib.ui',self)
        
        
        self.mode='manual'
        self.next_waypoint_index=0
        
        self.lat_list=[]
        self.long_list=[]
        
        self.waypoint_lat_list=[]
        self.waypoint_long_list=[]
        
        self.vx_list=[]
        self.vy_list=[]
        read_thread=threading.Thread(target=self.read_u_blox)
        read_thread=threading.Thread(target=self.simulate_read)
        
        
        read_thread.start()
        
        ## start plot loop. This cannot be in a seperate Thread since matplotlib can only be used in main thread
        #self.plot_long_lat_with_loop()
        self.init_plot_long_lat()
        
        self.update_simulation_button.clicked.connect(self.init_plot_long_lat)
        self.record_point_button.clicked.connect(self.record_point)
        
        self.timer=QTimer()
        self.timer.timeout.connect(self.update_plot_long_lat)
        self.timer.start(50)
        
        #self.update_simulation_button.clicked.connect(self.update_plot_long_lat)
        
        
    def on_manual_mode_button_clicked(self):
        self.manual_mode_button.setStyleSheet("background-color: green")
        self.auto_mode_button.setStyleSheet("background-color: None")
        self.mode='manual'
        
    def on_auto_mode_button_clicked(self):
        
        if len(self.waypoint_long_list)>0:
            self.manual_mode_button.setStyleSheet("background-color: None")
            self.auto_mode_button.setStyleSheet("background-color: green")
            self.mode='auto'
        
        
    def read_u_blox(self):
        
        
        stream = Serial('COM4', 9600, timeout=3)
        ubr = UBXReader(stream)

        while not keyboard.is_pressed('q'):
            (raw_data, parsed_data) = ubr.read()
            #print(parsed_data.identity)
            if parsed_data.identity == "NAV-PVT":
                #print(parsed_data)
                print('lat:',parsed_data.lat*1e-7)
                self.lat_list.append(parsed_data.lat*1e-7)
                print('lon:',parsed_data.lon*1e-7)
                self.long_list.append(parsed_data.lon*1e-7)
                print('height:',parsed_data.height*1e-3)
                print('height over mean sea level: ',parsed_data.hMSL*1e-3)
                print('velocity north: ',parsed_data.velN*1e-3)
                print('velocity east: ',parsed_data.velE*1e-3)
                print('velocity down: ',parsed_data.velD*1e-3)
   
    

        
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
            self.long_list.append(self.sim_x)            
            self.lat_list.append(self.sim_y)
            
            self.vx_list.append(self.sim_vx)
            self.vy_list.append(self.sim_vy)
            
          #  print('added lat, long ',self.lat_list[-1],self.long_list[-1])
          #  print('added vx, vy ',self.vx_list[-1],self.vy_list[-1])
            time.sleep(dt)
            
    def calculate_auto(self,delta_phi_turn=90/180*np.pi,d_min=0.1):
        self.d_min=d_min
        
        if self.next_waypoint_index >= len(self.waypoint_lat_list):
            self.next_waypoint_index =0
        
        
        ## calculate relative deviation        
        
        self.delta_long=self.waypoint_long_list[self.next_waypoint_index]-self.long_list[-1]
        self.delta_lat=self.waypoint_lat_list[self.next_waypoint_index]-self.lat_list[-1]
        
        self.delta_abs = np.sqrt(self.delta_lat**2+self.delta_long**2)
        
        ## calculate angle
        self.target_phi = np.arctan(self.delta_lat/self.delta_long)
        if self.delta_long<0 and self.delta_lat>0:
            self.target_phi+=np.pi
            
        if self.delta_long<0 and self.delta_lat<0:
            self.target_phi-=np.pi
         
        self.delta_phi=self.target_phi-self.sim_phi
        
        if self.delta_phi>np.pi:
            self.delta_phi-=2*np.pi
        if self.delta_phi<-np.pi:
            self.delta_phi+=2*np.pi
        
        
        
        
       # print('delta_lat',self.delta_lat,'delta_long',self.delta_long)
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
    
        
        
        
    def init_plot_long_lat(self,blit=True):
        self.blit=blit
        
        
        
        if len(self.lat_list)<2:
            self.points, = self.plot_widget.canvas.axes.plot([0,1], [0,1], 'ro')
            self.line, = self.plot_widget.canvas.axes.plot([0,1],[0,1],linewidth=1,color='lightblue')
            
            self.plot_widget.canvas.axes.set_xlim(-5, 5)
            self.plot_widget.canvas.axes.set_ylim(-5, 5)
        
        else:
            self.points, = self.plot_widget.canvas.axes.plot( self.long_list[-100::],self.lat_list[-100::], 'ro')
            self.line, = self.plot_widget.canvas.axes.plot( self.long_list[-100::],self.lat_list[-100::],linewidth=1,color='lightblue')
#            self.plot_widget.canvas.axes.relim()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
            
            self.plot_widget.canvas.axes.set_xlim(min(self.long_list)-1,max(self.long_list)+1)
            self.plot_widget.canvas.axes.set_ylim(min(self.lat_list)-1,max(self.lat_list)+1)
            self.plot_widget.canvas.draw()
        self.plot_widget.canvas.axes.set_xlabel(r'$long$')
        self.plot_widget.canvas.axes.set_ylabel(r'$lat$')
        
        
        self.arrow=self.plot_widget.canvas.axes.annotate('',xy=(0,1),xytext=(0,0),arrowprops={'arrowstyle':'->'})
        if blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
      
        
    def update_plot_long_lat(self):
        
        if len(self.lat_list)<100:
            self.points.set_data(self.long_list,self.lat_list)
            self.line.set_data(self.long_list,self.lat_list)
            
        else:
            self.points.set_data(self.long_list[-100::],self.lat_list[-100::])
            self.line.set_data(self.long_list[-100::],self.lat_list[-100::])
        if len(self.lat_list)>2:
           # self.plot_widget.canvas.axes.set_xlim(min(self.lat_list)-1,max(self.lat_list)+1)
           # self.plot_widget.canvas.axes.set_ylim(min(self.long_list)-1,max(self.long_list)+1)
            self.arrow.set_position((self.long_list[-1],self.lat_list[-1]))
            self.arrow.xy=(self.long_list[-1]+self.vx_list[-1],self.lat_list[-1]+self.vy_list[-1])
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
        self.waypoint_long_list.append(self.long_list[-1])
        
        self.waypoint_lat_list.append(self.lat_list[-1])
        
        
        self.waypoints, = self.plot_widget.canvas.axes.plot( self.waypoint_long_list,self.waypoint_lat_list,'ro',markersize=10,color='blue')
        self.plot_widget.canvas.axes.draw_artist(self.waypoints)
        
        if self.blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
        
        
        
        
            
        
      
            
            

if __name__=='__main__':
    
    
    app=QApplication(sys.argv)
    main = MainWindow()
    main.show()
    
    
    sys.exit(app.exec_())
    