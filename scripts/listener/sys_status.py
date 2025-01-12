#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray

import psutil
from pynvml import nvmlInit, nvmlDeviceGetHandleByIndex, nvmlDeviceGetUtilizationRates, nvmlShutdown


def get_sys_status(sys_status_pub):
    '''CPU usage, CPU frequency, GPU usage, memory usage'''
    
    #cpu usage
    cpu_usage = psutil.cpu_percent(interval=1)

    #cpu frequency
    cpu_freq_info = psutil.cpu_freq()
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
    finally:
        nvmlShutdown()

    #memory usage
    memory_percent = psutil.virtual_memory().percent

    print(f'CPU: {cpu_usage:.2f}%, Freq: {cpu_freq:.2f}%, GPU: {gpu_usage:.2f}%, Memory: {memory_percent:.2f}%')
    
    sys_status_msg = Float32MultiArray()
    sys_status_msg.data = [cpu_usage, cpu_freq, gpu_usage, memory_percent]
    sys_status_pub.publish(sys_status_msg)

def main():
    try:
        rospy.init_node('get_sys_status', anonymous=True)
        sys_status_pub = rospy.Publisher('/get_sys_status', Float32MultiArray, queue_size=10)
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            get_sys_status(sys_status_pub)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
