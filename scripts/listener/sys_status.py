#!/usr/bin/python3
# -*- coding: utf-8 -*-
import psutil
import subprocess
import pynvml

import rospy
from std_msgs.msg import Float32MultiArray, Float32

def get_gpu_usage():
    try:
        # 첫 번째 GPU 핸들 가져오기
        handle = pynvml.nvmlDeviceGetHandleByIndex(0)
        gpu_utilization = pynvml.nvmlDeviceGetUtilizationRates(handle).gpu  #gpu usage

        # 메모리 사용량
        mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
        mem_usage = (mem_info.total / 1024**2 / mem_info.used / 1024**2) * 100  #gpu memory usage
    except pynvml.NVMLError as e:
        print(f"Error monitoring GPU: {e}")
    
    return gpu_utilization, mem_usage
        
def sys_status():
    cpu_usage_pub = rospy.Publisher('/sys_status/cpu_usage', Float32, queue_size=10)
    cpu_freq_pub = rospy.Publisher('/sys_status/cpu_freq', Float32, queue_size=10)
    gpu_usage_pub = rospy.Publisher('/sys_status/gpu_usage', Float32, queue_size=10)
    memory_pub = rospy.Publisher('/sys_status/memory', Float32, queue_size=10)
    
    '''CPU usage, CPU frequency, GPU usage, memory usage'''
    
    '''cpu usage'''
    cpu_usage = psutil.cpu_percent(interval=1)

    '''cpu frequency'''
    cpu_freq_info = psutil.cpu_freq()
    if cpu_freq_info:
        cpu_freq = cpu_freq_info.current / 1000
    else:
        cpu_freq = 0.0

    '''gpu usage'''
    # try:
    #     result = subprocess.run(
    #         ['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
    #         stdout=subprocess.PIPE, text=True
    #     )
    #     gpu_usage = float(result.stdout.strip())
    # except Exception:
    #     gpu_usage = 0.0
    pynvml.nvmlInit()
    gpu_usage, gpu_mem_usage = get_gpu_usage()
        
    '''memory usage'''
    memory_percent = psutil.virtual_memory().percent

    print(cpu_usage)
    print(cpu_freq)
    print(gpu_usage)
    print(memory_percent)
    
    cpu_usage_pub.publish(cpu_usage)
    cpu_freq_pub.publish(cpu_freq)
    gpu_usage_pub.publish(gpu_usage)
    memory_pub.publish(memory_percent)

if __name__ == '__main__':
    try:
        rospy.init_node('sys_status', anonymous=True)
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            sys_status()
            rate.sleep()
        sys_status()

    except rospy.ROSInterruptException:
        pass
