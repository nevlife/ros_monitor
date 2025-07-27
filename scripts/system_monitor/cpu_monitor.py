#!/usr/bin/env python3
import psutil
import os

def get_cpu_info():
    """
    returns a dictionary with CPU usage, temperature, load average, core count, logical count, and frequency.
    Returns:
        {
            'usage_percent': float,     # CPU 사용률 (%)
            'temperature': float,       # CPU 온도 (°C), 없으면 -1
            'load_average': list,       # 로드 평균 [1분, 5분, 15분]
            'core_count': int,          # 물리 코어 수
            'logical_count': int,       # 논리 코어 수
            'frequency': float          # 현재 주파수 (MHz)
        }
    """
    usage_percent = psutil.cpu_percent(interval=1)
    
    try:
        temp_sensors = psutil.sensors_temperatures()
        if 'coretemp' in temp_sensors:
            temperature = temp_sensors['coretemp'][0].current
        else:
            temperature = -1.0
    except (AttributeError, IndexError):
        temperature = -1.0
    
    try:
        load_average = list(os.getloadavg())
    except OSError:  # for Windows or unsupported systems
        load_average = [-1, -1, -1]
    
    # count of large and logical cores
    core_count = psutil.cpu_count(logical=False)
    logical_count = psutil.cpu_count(logical=True)
    
    # CPU frequency
    try:
        freq = psutil.cpu_freq()
        frequency = freq.current if freq else -1.0
    except:
        frequency = -1.0
    
    return {
        'usage_percent': usage_percent,
        'temperature': temperature,
        'load_average': load_average,
        'core_count': core_count,
        'logical_count': logical_count,
        'frequency': frequency
    }
