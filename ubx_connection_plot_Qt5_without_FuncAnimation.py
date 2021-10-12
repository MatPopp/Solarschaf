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
        
        
        
        self.lat_list=[]
        self.long_list=[]
        
        self.waypoint_lat_list=[]
        self.waypoint_long_list=[]
        
        self.vx_list=[]
        self.vy_list=[]
        read_thread=threading.Thread(target=self.read_u_blox)
        read_thread=threading.Thread(target=self.simulate_read_with_keyboard)
        read_thread.start()
        
        ## start plot loop. This cannot be in a seperate Thread since matplotlib can only be used in main thread
        #self.plot_lat_long_with_loop()
        self.init_plot_lat_long()
        
        self.update_simulation_button.clicked.connect(self.init_plot_lat_long)
        self.record_point_button.clicked.connect(self.record_point)
        
        self.timer=QTimer()
        self.timer.timeout.connect(self.update_plot_lat_long)
        self.timer.start(10)
        
        #self.update_simulation_button.clicked.connect(self.update_plot_lat_long)
        
        
    def on_manual_mode_button_clicked(self):
        self.manual_mode_button.setStyleSheet("background-color: green")
        self.auto_mode_button.setStyleSheet("background-color: None")
        
    def on_auto_mode_button_clicked(self):
        self.manual_mode_button.setStyleSheet("background-color: None")
        self.auto_mode_button.setStyleSheet("background-color: green")
        
        
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
        
        dt=1
        self.sim_vx=1
        self.sim_vy=1
        self.sim_x=0
        self.sim_y=0
        
        ## random walk
        while not keyboard.is_pressed('q'):
            
            self.sim_x+=self.sim_vx
            self.sim_y+=self.sim_vy
            
            self.sim_vx=np.random.randn()
            self.sim_vy=np.random.randn()
            
            self.lat_list.append(self.sim_x)
            self.long_list.append(self.sim_y)
            
            self.vx_list.append(self.sim_vx)
            self.vy_list.append(self.sim_vy)
            
            print('added lat, long ',self.lat_list[-1],self.long_list[-1])
            print('added vx, vy ',self.vx_list[-1],self.vy_list[-1])
            time.sleep(dt)
        
    def simulate_read_with_keyboard(self):
        
        dt=0.1
        self.sim_v=0
        self.sim_omega = 0
        self.sim_x=0
        self.sim_y=0
        self.sim_phi = 0
        self.sim_gamma_v = 5
        self.sim_gamma_omega = 10
        
        self.sim_mass=100
        self.sim_m_o_inertia = 10
        
        self.stdev=1 ### standard deviation of measurement values in m 
        
        motor_speed=1
        steering_ratio=1
        set_left=0
        set_right=0
        
        ## random walk
        while not keyboard.is_pressed('q'):
            
            ## check keyboard and send messages accordingly
            
                
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
            
            self.sim_v += (set_left+set_right)/self.sim_mass - self.sim_v*self.sim_gamma_v/self.sim_mass
            self.sim_omega += (set_right-set_left)/self.sim_m_o_inertia - self.sim_omega*self.sim_gamma_omega/self.sim_m_o_inertia

            
            self.sim_vx=self.sim_v*np.cos(self.sim_phi)
            self.sim_vy=self.sim_v*np.sin(self.sim_phi)
            
            
            self.sim_x+=self.sim_vx
            self.sim_y+=self.sim_vy
            self.sim_phi +=self.sim_omega
            
            self.lat_list.append(self.sim_x)
            self.long_list.append(self.sim_y)
            
            self.vx_list.append(self.sim_vx)
            self.vy_list.append(self.sim_vy)
            
          #  print('added lat, long ',self.lat_list[-1],self.long_list[-1])
          #  print('added vx, vy ',self.vx_list[-1],self.vy_list[-1])
            time.sleep(dt)
            
        
    def init_plot_lat_long(self,blit=True):
        self.blit=blit
        
        
        
        if len(self.lat_list)<2:
            self.points, = self.plot_widget.canvas.axes.plot([0,1], [0,1], 'ro')
            self.line, = self.plot_widget.canvas.axes.plot([0,1],[0,1],linewidth=1,color='lightblue')
            
            self.plot_widget.canvas.axes.set_xlim(-5, 5)
            self.plot_widget.canvas.axes.set_ylim(-5, 5)
        
        else:
            self.points, = self.plot_widget.canvas.axes.plot(self.lat_list[-100::], self.long_list[-100::], 'ro')
            self.line, = self.plot_widget.canvas.axes.plot(self.lat_list[-100::], self.long_list[-100::],linewidth=1,color='lightblue')
#            self.plot_widget.canvas.axes.relim()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
#            self.plot_widget.canvas.axes.autoscale_view()
            
            self.plot_widget.canvas.axes.set_xlim(min(self.lat_list)-1,max(self.lat_list)+1)
            self.plot_widget.canvas.axes.set_ylim(min(self.long_list)-1,max(self.long_list)+1)
            self.plot_widget.canvas.draw()
        self.plot_widget.canvas.axes.set_xlabel(r'$x$')
        self.plot_widget.canvas.axes.set_ylabel(r'$y$')
        print('bin vor Funk def')
        
        
        self.arrow=self.plot_widget.canvas.axes.annotate('',xy=(0,1),xytext=(0,0),arrowprops={'arrowstyle':'->'})
        if blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
      
        
    def update_plot_lat_long(self):
        print('bin in update')
        
        if len(self.lat_list)<100:
            self.points.set_data(self.lat_list, self.long_list)
            self.line.set_data(self.lat_list, self.long_list)
            
        else:
            self.points.set_data(self.lat_list[-100::], self.long_list[-100::])
            self.line.set_data(self.lat_list[-100::], self.long_list[-100::])
        if len(self.lat_list)>2:
           # self.plot_widget.canvas.axes.set_xlim(min(self.lat_list)-1,max(self.lat_list)+1)
           # self.plot_widget.canvas.axes.set_ylim(min(self.long_list)-1,max(self.long_list)+1)
            self.arrow.set_position((self.lat_list[-1],self.long_list[-1]))
            self.arrow.xy=(self.lat_list[-1]+self.vx_list[-1],self.long_list[-1]+self.vy_list[-1])
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
        self.waypoint_lat_list.append(self.lat_list[-1])
        self.waypoint_long_list.append(self.long_list[-1])
        
        
        self.waypoints, = self.plot_widget.canvas.axes.plot(self.waypoint_lat_list, self.waypoint_long_list,'ro',markersize=10,color='blue')
        self.plot_widget.canvas.axes.draw_artist(self.waypoints)
        
        if self.blit:
            # cache the background
            self.axbackground = self.plot_widget.canvas.copy_from_bbox(self.plot_widget.canvas.axes.bbox)
        
        
        
        
            
        
      
            
            

if __name__=='__main__':
    
    
    app=QApplication(sys.argv)
    main = MainWindow()
    main.show()
    
    
    sys.exit(app.exec_())
    