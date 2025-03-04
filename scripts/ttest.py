#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import re
import rosnode
import rospy
import pynvml
from xmlrpc.client import ServerProxy

def find_ros_node_pid(node_name):
    """
    /proc 디렉토리를 검색하여, 해당 노드 이름이 포함된 cmdline을 가진 프로세스의 PID를 반환합니다.
    """
    for pid in os.listdir("/proc"):
        if pid.isdigit():
            try:
                with open(f"/proc/{pid}/cmdline", "r") as f:
                    cmd = f.read()
                if node_name in cmd:
                    return int(pid)
            except Exception:
                continue
    return 0

def get_node_pid(node_name):
    """
    rosnode.get_api_uri()를 통해 해당 노드의 API URI를 얻고, XMLRPC로 getPid()를 호출하여 PID를 가져옵니다.
    실패할 경우 /proc에서 직접 검색합니다.
    """
    try:
        # caller_id와 target node를 함께 전달
        node_uri = rosnode.get_api_uri(rospy.get_name(), node_name)
    except Exception as e:
        rospy.logwarn("Failed to lookup node URI for {}: {}".format(node_name, e))
        return find_ros_node_pid(node_name)
    
    if not node_uri:
        rospy.logwarn("Empty node URI for {}".format(node_name))
        return find_ros_node_pid(node_name)
    
    # URI 형식 예: "http://hostname:port/"
    m = re.match(r"http://([^:]+):(\d+)/", node_uri)
    if not m:
        rospy.logwarn("Failed to parse node URI for {}: {}".format(node_name, node_uri))
        return find_ros_node_pid(node_name)
    host, port_str = m.group(1), m.group(2)
    try:
        port = int(port_str)
    except Exception:
        rospy.logwarn("Invalid port in node URI for {}: {}".format(node_name, node_uri))
        return find_ros_node_pid(node_name)
    
    try:
        client = ServerProxy("http://{}:{}/".format(host, port))
        result = client.getPid(rospy.get_name())
        # result는 보통 [statusCode, statusMessage, pid]
        if isinstance(result, list) and len(result) >= 3:
            return int(result[2])
    except Exception as e:
        rospy.logwarn("Failed to call getPid on node {}: {}".format(node_name, e))
        return find_ros_node_pid(node_name)
    
    rospy.logwarn("Invalid getPid response from node {}".format(node_name))
    return find_ros_node_pid(node_name)

def get_all_ros_pids():
    """
    rosnode.get_node_names()를 사용하여 ROS에 등록된 모든 노드의 PID 집합을 반환합니다.
    """
    pids = set()
    for node in rosnode.get_node_names():
        pid = get_node_pid(node)
        if pid:
            pids.add(pid)
    return pids

def get_gpu_usage_by_pids(pids):
    """
    이미 NVML이 초기화된 상태에서, 입력받은 PID 집합에 해당하는 GPU 프로세스 정보를 조회합니다.
    반환값은 각 PID에 대해 GPU 사용 정보를 딕셔너리 형태로 저장한 자료입니다.
    """
    usage_info = {}
    try:
        device_count = pynvml.nvmlDeviceGetCount()
    except pynvml.NVMLError as e:
        rospy.logwarn("Failed to get GPU count: {}".format(e))
        return usage_info

    for i in range(device_count):
        try:
            dev = pynvml.nvmlDeviceGetHandleByIndex(i)
            dev_name = pynvml.nvmlDeviceGetName(dev)
            util = pynvml.nvmlDeviceGetUtilizationRates(dev)
        except pynvml.NVMLError as e:
            rospy.logwarn("Error retrieving GPU {} info: {}".format(i, e))
            continue

        # Compute와 Graphics 프로세스 모두 조회
        try:
            procs_compute = pynvml.nvmlDeviceGetComputeRunningProcesses(dev)
        except pynvml.NVMLError:
            procs_compute = []
        try:
            procs_graphics = pynvml.nvmlDeviceGetGraphicsRunningProcesses(dev)
        except pynvml.NVMLError:
            procs_graphics = []
        all_procs = procs_compute + procs_graphics

        for proc in all_procs:
            if proc.pid in pids:
                used_mem = proc.usedGpuMemory / (1024*1024)  # MiB 단위
                info = {
                    "gpu_index": i,
                    "gpu_name": dev_name.decode() if isinstance(dev_name, bytes) else dev_name,
                    "used_mem_MiB": used_mem,
                    "gpu_utilization": util.gpu,
                    "mem_utilization": util.memory
                }
                if proc.pid in usage_info:
                    usage_info[proc.pid].append(info)
                else:
                    usage_info[proc.pid] = [info]
    return usage_info

def main():
    rospy.init_node("ros_gpu_usage_monitor", anonymous=False)
    rate = rospy.Rate(1)  # 1Hz

    # NVML 초기화 (함수 밖에서 처리)
    try:
        pynvml.nvmlInit()
    except pynvml.NVMLError as e:
        rospy.logerr("NVML initialization failed: {}".format(e))
        return

    rospy.loginfo("Monitoring GPU usage for all ROS processes...")

    while not rospy.is_shutdown():
        ros_pids = get_all_ros_pids()
        gpu_usage = get_gpu_usage_by_pids(ros_pids)
        rospy.loginfo("GPU usage for ROS processes:")
        for pid in ros_pids:
            usage = gpu_usage.get(pid, "No GPU usage")
            rospy.loginfo("PID {}: {}".format(pid, usage))
        rate.sleep()

    pynvml.nvmlShutdown()

if __name__ == "__main__":
    main()
