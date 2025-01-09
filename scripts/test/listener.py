#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber

def sys_status_callback(
    cpu_usage, 
    cpu_freq, 
    gpu_usage, 
    memory
    ):
    cpu_usage = cpu_usage.data
    cpu_freq = cpu_freq.data
    gpu_usage = gpu_usage.data
    memory = memory.data
    
    print(f"CPU usage: {cpu_usage}%")
    print(f"CPU frequency: {cpu_freq}GHz")
    print(f"GPU usage: {gpu_usage}%")
    print(f"Memory usage: {memory}%")

def main():
    rospy.init_node('listener_node', anonymous=True)
    
    cpu_usage = Subscriber('/sys_status/cpu_usage', Float32)
    cpu_freq = Subscriber('/sys_status/cpu_freq', Float32)
    gpu_usage = Subscriber('/sys_status/gpu_usage', Float32)
    memory = Subscriber('/sys_status/memory', Float32)
    
    ats = ApproximateTimeSynchronizer(
        [cpu_usage, cpu_freq, gpu_usage, memory],
        queue_size=10,
        slop=1, 
        allow_headerless=True
    )
    ats.registerCallback(sys_status_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass