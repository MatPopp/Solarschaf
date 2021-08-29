# -*- coding: utf-8 -*-
"""
Created on Tue Aug 24 20:48:15 2021

@author: matth
"""

from serial import Serial
from pyubx2 import UBXReader
import keyboard

stream = Serial('/dev/ttyACM0', 9600, timeout=3)
ubr = UBXReader(stream)

while True:
    (raw_data, parsed_data) = ubr.read()
    #print(parsed_data)
    if parsed_data.identity == "NAV-PVT":
        #print(parsed_data)
        print('lat:',parsed_data.lat*1e-7)
        print('lon:',parsed_data.lon*1e-7)
        print('height:',parsed_data.height*1e-3)
        print('height over mean sea level: ',parsed_data.hMSL*1e-3)
        print('velocity north: ',parsed_data.velN*1e-3)
        print('velocity east: ',parsed_data.velE*1e-3)
        print('velocity down: ',parsed_data.velD*1e-3)
    

