#!/usr/bin/env python3
import os
import sys
import time
import psutil
import pynvml
import rospy
from std_msgs.msg import Float32MultiArray

scripts_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(scripts_dir)

from core.core import Monitor, AggregationStrategies

class SystemStatusMonitor:
    def __init__(self):
        rospy.init_node("system_resource_node", anonymous=True)
        self.rate = rospy.Rate(1)  # 1Hz
        
        # 기존 퍼블리셔 (필요하면 유지)
        #self.publisher = rospy.Publisher("/total_resource", Float32MultiArray, queue_size=100)
        
        # Monitor 객체 생성
        self.monitor = Monitor(monitorDescription="System Resource Monitor", monitoring_type="system_resource")

        self.gpu_initialized = False
        try:
            pynvml.nvmlInit()
            self.gpu_handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            gpu_name = pynvml.nvmlDeviceGetName(self.gpu_handle)
            self.gpu_initialized = True
            rospy.loginfo(f"NVML initialized GPU: {gpu_name}")
        except pynvml.NVMLError as e:
            rospy.logwarn(f"GPU initialization failed: {e}")

        self.prev_cpu_times = psutil.cpu_times()

    def run(self):
        while not rospy.is_shutdown():
            # 기존 Float32MultiArray 메시지 생성 (필요하면 유지)
            # msg = Float32MultiArray()

            # 시스템 리소스 정보 수집
            cpu_info = self.get_cpu_info()
            mem_info = self.get_memory_usage()
            gpu_info = self.get_gpu_info()

            # 기존 메시지에 데이터 추가 (필요하면 유지)
            # msg.data.extend(cpu_info)
            # msg.data.extend(mem_info)
            # msg.data.extend(gpu_info)
            # self.publisher.publish(msg)
            tmp_target = -1
            # Monitor를 사용하여 데이터 추가
            # CPU 정보
            self.monitor.addValue("cpu_usage", cpu_info[10], "%", tmp_target, 0)  # CPU 사용률
            self.monitor.addValue("cpu_temp", cpu_info[11], "°C", tmp_target, 0 if cpu_info[11] < 80 else 2)  # CPU 온도
            self.monitor.addValue("load_avg_1min", cpu_info[12], "", tmp_target, 0 if cpu_info[12] < 1 else 1)  # 1분 로드 평균
            self.monitor.addValue("load_avg_5min", cpu_info[13], "", tmp_target, 0 if cpu_info[12] < 1 else 1) 
            self.monitor.addValue("load_avg_15min", cpu_info[14], "", tmp_target, 0 if cpu_info[12] < 1 else 1) 
            
            # 메모리 정보
            self.monitor.addValue("mem_used", mem_info[0], "MB", tmp_target, 0)  # 사용 메모리
            self.monitor.addValue("mem_total", mem_info[1], "MB", tmp_target, 0)  # 전체 메모리
            self.monitor.addValue("mem_usage_percent", mem_info[2], "%", tmp_target, 
                                 0 if mem_info[2] < 80 else (1 if mem_info[2] < 90 else 2))  # 메모리 사용률
            
            # GPU 정보 (GPU가 있는 경우)
            if self.gpu_initialized and gpu_info[0] >= 0:
                self.monitor.addValue("gpu_usage", gpu_info[0], "%", tmp_target, 0 if gpu_info[0] < 80 else 1)  # GPU 사용률
                self.monitor.addValue("gpu_mem_used", gpu_info[1], "MB", tmp_target, 0)  # GPU 메모리 사용량
                self.monitor.addValue("gpu_mem_total", gpu_info[2], "MB", tmp_target, 0)  # GPU 전체 메모리
                self.monitor.addValue("gpu_mem_usage_percent", gpu_info[3], "%", tmp_target, 
                                     0 if gpu_info[3] < 80 else (1 if gpu_info[3] < 90 else 2))  # GPU 메모리 사용률
                self.monitor.addValue("gpu_temp", gpu_info[4], "C", tmp_target,
                                     0 if gpu_info[4] < 75 else (1 if gpu_info[4] < 85 else 2))  # GPU 온도

            # 콘솔 출력 (디버깅용)
            print(f"CPU: {cpu_info}")
            print(f"Mem: {mem_info}")
            if self.gpu_initialized:
                print(f"GPU: {gpu_info}")

            self.rate.sleep()

    def get_cpu_info(self):
        cpu_times = psutil.cpu_times()
        total_time = sum(cpu_times)
        idle_time = cpu_times.idle

        total_diff = total_time - sum(self.prev_cpu_times)
        idle_diff = idle_time - self.prev_cpu_times.idle
        cpu_usage_percent = (total_diff - idle_diff) / total_diff * 100 if total_diff else 0

        self.prev_cpu_times = cpu_times

        load_avg = os.getloadavg()

        try:
            temp = psutil.sensors_temperatures().get("coretemp", [])[0].current
        except (AttributeError, IndexError):
            temp = -1.0

        return [
            cpu_times.user, cpu_times.nice, cpu_times.system, cpu_times.idle,
            cpu_times.iowait, cpu_times.irq, cpu_times.softirq, cpu_times.steal,
            cpu_times.guest, cpu_times.guest_nice,
            cpu_usage_percent,
            temp,
            load_avg[0], load_avg[1], load_avg[2]
        ]

    def get_memory_usage(self):
        mem = psutil.virtual_memory()
        mem_used = (mem.total - mem.available) / (1024 * 1024)
        mem_total = mem.total / (1024 * 1024)
        mem_usage_percent = mem_used / mem_total * 100
        return [mem_used, mem_total, mem_usage_percent]

    def get_gpu_info(self):
        if not self.gpu_initialized:
            return [-1, -1, -1, -1, -1]

        try:
            utilization = pynvml.nvmlDeviceGetUtilizationRates(self.gpu_handle)
            memory = pynvml.nvmlDeviceGetMemoryInfo(self.gpu_handle)
            temperature = pynvml.nvmlDeviceGetTemperature(self.gpu_handle, pynvml.NVML_TEMPERATURE_GPU)

            gpu_usage_percent = utilization.gpu
            gpu_mem_used = memory.used / (1024 * 1024)
            gpu_mem_total = memory.total / (1024 * 1024)
            gpu_mem_usage = (gpu_mem_used / gpu_mem_total) * 100
            gpu_temp = temperature

            return [gpu_usage_percent, gpu_mem_used, gpu_mem_total, gpu_mem_usage, gpu_temp]
        except pynvml.NVMLError as e:
            rospy.logwarn(f"Failed to fetch GPU info: {e}")
            return [-1, -1, -1, -1, -1]


if __name__ == "__main__":
    monitor = SystemStatusMonitor()

    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass