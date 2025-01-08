#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

import subprocess

def main():
    process1 = subprocess.Popen(['rosrun', 'vehicle_diag', 'sys_status_monitor.py'], 
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print('Starting sys_status_monitor.py')

    process2 = subprocess.Popen(['rosrun', 'vehicle_diag', 'gui.py'], 
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print('Starting gui.py')
    
    process3 = subprocess.Popen(['rosrun', 'vehicle_diag', 'sensor_status.py'], 
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print('Starting sensor_status.py')
    
if __name__ == '__main__':
    main()