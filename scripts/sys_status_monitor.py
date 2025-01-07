#!/usr/bin/python3
# -*- coding: utf-8 -*-
import psutil
import rospy
from std_msgs.msg import Float32MultiArray, Float32
import subprocess

def sys_monitor():
    cpu_usage_pub = rospy.Publisher('/sys_status/cpu_usage', Float32, queue_size=10)
    cpu_freq_pub = rospy.Publisher('/sys_status/cpu_freq', Float32, queue_size=10)
    gpu_usage_pub = rospy.Publisher('/sys_status/gpu_usage', Float32, queue_size=10)
    memory_pub = rospy.Publisher('/sys_status/memory', Float32, queue_size=10)
    battery_pub = rospy.Publisher('/sys_status/battery', Float32, queue_size=10)
    
    '''CPU usage, CPU frequency, GPU usage, memory usage, battery percentage'''
    
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
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
            stdout=subprocess.PIPE, text=True
        )
        gpu_usage = float(result.stdout.strip())
    except Exception:
        gpu_usage = 0.0

    #memory usage
    memory_percent = psutil.virtual_memory().percent

    #battery percentage
    try:
        battery = psutil.sensors_battery()
        battery_percent = battery.percent if battery else 0.0
    except Exception:
        battery_percent = 0.0

    cpu_usage_pub.publish(cpu_usage)
    cpu_freq_pub.publish(cpu_freq)
    gpu_usage_pub.publish(gpu_usage)
    memory_pub.publish(memory_percent)
    battery_pub.publish(battery_percent)

if __name__ == '__main__':
    try:
        rospy.init_node('sys_status', anonymous=True)
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            sys_monitor()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
