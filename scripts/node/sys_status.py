#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray

from psutil import cpu_percent, cpu_freq, virtual_memory
from pynvml import nvmlInit, nvmlDeviceGetHandleByIndex, nvmlDeviceGetUtilizationRates, nvmlShutdown


def get_sys_status():
    sys_status_pub = rospy.Publisher('/sys_status', Float32MultiArray, queue_size=10)

    '''CPU usage, CPU frequency, GPU usage, memory usage'''
    
    #cpu usage
    cpu_usage = cpu_percent(interval=1)

    #cpu frequency
    cpu_freq_info = cpu_freq()
    if cpu_freq_info:
        cpu_freq = cpu_freq_info.current / 1000
    else:
        cpu_freq = 0.0

    #gpu usage
    try:
        nvmlInit()
        gpu_usage = nvmlDeviceGetUtilizationRates(nvmlDeviceGetHandleByIndex(0)).gpu
        nvmlShutdown()
    except Exception:
        gpu_usage = 0.0

    #memory usage
    memory_percent = virtual_memory().percent

    sys_status_msg = Float32MultiArray()
    sys_status_msg.data = [cpu_usage, cpu_freq, gpu_usage, memory_percent]
    sys_status_pub.publish(sys_status_msg)

def main():
    try:
        rospy.init_node('sys_status', anonymous=True)
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            get_sys_status()
            rate.sleep()
        get_sys_status()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
