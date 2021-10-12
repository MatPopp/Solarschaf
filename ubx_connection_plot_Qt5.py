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

    
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)


class MainWindow(QMainWindow):
    def __init__(self,*args, **kwargs):
        super(MainWindow, self).__init__(*args,**kwargs)
        
        loadUi('QtMatplotlib.ui',self)
        
        
        
        self.lat_list=[]
        self.long_list=[]
        
        self.vx_list=[]
        self.vy_list=[]
        read_thread=threading.Thread(target=self.read_u_blox)
        read_thread=threading.Thread(target=self.simulate_read_with_keyboard)
        read_thread.start()
        
        ## start plot loop. This cannot be in a seperate Thread since matplotlib can only be used in main thread
        #self.plot_lat_long_with_loop()
        self.plot_lat_long_with_animation()
        
        
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
        
        dt=0.2
        self.sim_v=0
        self.sim_omega = 0
        self.sim_x=0
        self.sim_y=0
        self.sim_phi = 0
        self.sim_gamma_v = 5
        self.sim_gamma_omega = 10
        
        self.sim_mass=100
        self.sim_m_o_inertia = 10
        
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
            
        
    def plot_lat_long_with_animation(self):
        
        
        points, = self.plot_widget.canvas.axes.plot([0,1], [0,1], 'ro')
        line, = self.plot_widget.canvas.axes.plot([0,1],[0,1],linewidth=1,color='lightblue')
        self.plot_widget.canvas.axes.set_xlabel(r'$x$')
        self.plot_widget.canvas.axes.set_ylabel(r'$y$')
        print('bin vor Funk def')
        
            
        def init():
            print('bin in init')
            self.plot_widget.canvas.axes.set_xlim(0, 1)
            self.plot_widget.canvas.axes.set_ylim(0, 1)
            self.arrow=self.plot_widget.canvas.axes.annotate('',xy=(0,1),xytext=(0,0),arrowprops={'arrowstyle':'->'})
            return points, line, self.arrow
      
        
        def update(frame):
            print('bin in update')
            
            points.set_data(self.lat_list, self.long_list)
            line.set_data(self.lat_list, self.long_list)
            if len(self.lat_list)>2:
                self.plot_widget.canvas.axes.set_xlim(min(self.lat_list)-1,max(self.lat_list)+1)
                self.plot_widget.canvas.axes.set_ylim(min(self.long_list)-1,max(self.long_list)+1)
                self.arrow.set_position((self.lat_list[-1],self.long_list[-1]))
                self.arrow.xy=(self.lat_list[-1]+self.vx_list[-1],self.long_list[-1]+self.vy_list[-1])
                self.plot_widget.fig.canvas.draw()
            return points,line,self.arrow
        
        print('bin vor ani def')
        self.ani = FuncAnimation(self.plot_widget.fig, update,interval=100,
                            init_func=init, blit=True)
        print('bin nach ani')
        #plt.show()
            
            

if __name__=='__main__':
    
    
    app=QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())