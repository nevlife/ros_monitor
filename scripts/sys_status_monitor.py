#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import psutil
import subprocess
def sys_monitor():
    cpu_usage = psutil.cpu_percent(interval=1)
    core_usage = psutil.cpu_percent(interval=1, percpu=True)
    
    gpu_usage = subprocess.run(
        ['nvidia-smi',
         '--query-gpu=utilization.gpu', 
         '--format=csv,noheader,nounits'
         ],
        stdout=subprocess.PIPE,
        text=True
    )
if __name__ == '__main__':
    try:
        rospy.init_node('sys_status_monitor')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass