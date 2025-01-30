#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
import psutil
from pynvml import nvmlInit, nvmlDeviceGetHandleByIndex, nvmlDeviceGetUtilizationRates, nvmlDeviceGetMemoryInfo, nvmlDeviceGetTemperature, nvmlDeviceGetName, nvmlShutdown, NVML_TEMPERATURE_GPU, NVMLError, nvmlDeviceGetCount


class SystemStatusMonitor:
    def __init__(self):
        """Initialize the System Status Monitor."""
        # ROS Publisher 초기화
        self.total_resource_pub = rospy.Publisher('/total_resource', Float32MultiArray, queue_size=100)
        self.rate = rospy.Rate(1)  # 1Hz

        # GPU 초기화
        self.gpu_initialized = False
        try:
            nvmlInit()
            self.gpu_handle = nvmlDeviceGetHandleByIndex(0)
            self.gpu_initialized = True
            rospy.loginfo(f"NVML initialized GPU: {nvmlDeviceGetName(self.gpu_handle)}")
        except NVMLError as e:
            rospy.logwarn(f"GPU initialization failed: {e}")

    def get_cpu_usage(self):
        '''
        psutil.cpu_times_percent return:
        (user, nice, system, idle, iowait, irq, softirq, steal, guset, guset_nice)
        '''
        cpu_usage_total_percent = psutil.cpu_percent(interval=1)
        cpu_usage = psutil.cpu_times_percent(interval=1) #Linux
        
        cpu_freq = psutil.cpu_freq().current / 1000 if psutil.cpu_freq() else -1
        try:
            temps = psutil.sensors_temperatures()
            if 'coretemp' in temps: #x86 cpu
                core_temps = temps['coretemp']
                cpu_temp = sum([t.current for t in core_temps]) / len(core_temps)
            elif 'cpu-thermal' in temps: #arm cpu
                cpu_temp = temps['cpu-thermal'][0].current
            else:
                cpu_temp = -1
        except Exception as e:
            cpu_temp = -1
        return [cpu_usage, cpu_freq, cpu_temp, cpu_usage_total_percent]

    def get_gpu_info(self):
        """return gpu info"""
        if not self.gpu_initialized:
            return []
        
        gpu_info_lst = []
        
        try:
            gpu_name = nvmlDeviceGetName(self.gpu_handle)
            gpu_usage = nvmlDeviceGetUtilizationRates(self.gpu_handle).gpu
            gpu_memory = nvmlDeviceGetMemoryInfo(self.gpu_handle)
            gpu_temp = nvmlDeviceGetTemperature(self.gpu_handle, NVML_TEMPERATURE_GPU)
            
            gpu_info_lst.append([
                0, #id
                gpu_name, #model
                gpu_usage, #gpu usage percent
                gpu_memory.used / (1024**2), #gpu used mem (mb)
                gpu_memory.total / (1024**2), #gpu total mem (mb)
                (gpu_memory.used / gpu_memory.total) * 100 if gpu_memory.total > 0 else -1, #gpu used mem (%)
                gpu_temp #gpu temperature
            ])
                
        except Exception as e:
            rospy.logwarn(f"Failed to get GPU usage: {e}")
            
        return gpu_info_lst
    
    def get_memory_usage(self):
        """Get system memory usage."""
        memory = psutil.virtual_memory()
        return [
            memory.used / (1024**2), #ram used (mb)
            memory.total / (1024**2), #ram total size (mb)
            memory.percent, #ram used (%)
        ]


    def run(self):
        """Main loop to continuously monitor and publish system status."""
        while not rospy.is_shutdown():
            cpu_info = self.get_cpu_usage()
            gpu_info = self.get_gpu_info()
            mem_info = self.get_memory_usage()
            
            msg = Float32MultiArray()
            
            msg.data = cpu_info + mem_info
            
            for gpu in gpu_info:
                msg.data.extend([
                    gpu[0], #gpu id
                    gpu[2], #gpu usage %
                    gpu[3], #gpu used mem mb
                    gpu[4], #gpu total mem mb
                    gpu[5], #gpu mem usage %
                    gpu[6], #gpu temp
                ])
                
            self.total_resource_pub.publish(msg)
            
            rospy.loginfo(f'CPU: user {cpu_info[0][0]:.2f}, system {cpu_info[0][2]}, idle {cpu_info[0][3]}, iowait {cpu_info[0][4]}'
                          f'| Freq: {cpu_info[1]:.2f}GHz | Temp: {cpu_info[2]:.2f}C')
            
            rospy.loginfo(f'Mem: {mem_info[0]:.2f}/{mem_info[1]:.2f} MB ({mem_info[2]:.2f}%)')

            # rospy.loginfo(f'GPU {gpu[0]:.2f}: {gpu[1]} | Usage: {gpu[2]:.2f}% | '
            #               f'GPU Mem: {gpu[3]:.2f}/{gpu[4]:.2f} MB {gpu[5]:.2f}% | Temp: {gpu[6]:.2f}°C')
            rospy.loginfo(f'GPU: {gpu[2]:.2f}% | GPU Mem: {gpu[3]:.2f}/{gpu[4]:.2f} MB ({gpu[5]:.2f}%) | Temp: {gpu[6]:.2f}C')
            
            self.rate.sleep()
            
    def shutdown(self):
        """Clean up resources on shutdown."""
        if self.gpu_initialized:
            nvmlShutdown()
            rospy.loginfo("GPU resources released.")


def main():
    rospy.init_node('total_resouce', anonymous=True)
    monitor = SystemStatusMonitor()

    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        monitor.shutdown()


if __name__ == '__main__':
    main()