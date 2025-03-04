#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import re
import rosnode
import rospy
import pynvml
import psutil
import json
from xmlrpc.client import ServerProxy
from std_msgs.msg import String

def find_ros_node_pid(node_name):
    """
    /proc에서 cmdline을 검색하여 ROS 노드 이름이 포함된 프로세스의 PID를 반환.
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
    ROS 마스터를 통해 노드 API URI를 가져와 XMLRPC로 getPid() 요청.
    실패하면 /proc에서 검색.
    """
    system_nodes = ["/rosout", rospy.get_name()]
    if node_name in system_nodes:
        rospy.logdebug("Skipping system node {}".format(node_name))
        return 0

    try:
        node_uri = rosnode.get_api_uri(rospy.get_name(), node_name)
    except Exception as e:
        rospy.logwarn("Failed to lookup node URI for {}: {}. Falling back to /proc search.".format(node_name, e))
        return find_ros_node_pid(node_name)
    
    if not node_uri:
        rospy.logwarn("Empty node URI for {}. Falling back to /proc search.".format(node_name))
        return find_ros_node_pid(node_name)
    
    # API URI 분석
    m = re.match(r"http://([^:]+):(\d+)/", node_uri)
    if not m:
        rospy.logwarn("Failed to parse node URI for {}: {}. Falling back to /proc search.".format(node_name, node_uri))
        return find_ros_node_pid(node_name)
    host, port_str = m.group(1), m.group(2)
    try:
        port = int(port_str)
    except Exception:
        rospy.logwarn("Invalid port in node URI for {}: {}. Falling back to /proc search.".format(node_name, node_uri))
        return find_ros_node_pid(node_name)
    
    try:
        client = ServerProxy("http://{}:{}/".format(host, port))
        result = client.getPid(rospy.get_name())
        if isinstance(result, list) and len(result) >= 3:
            return int(result[2])
    except Exception as e:
        rospy.logwarn("Failed to call getPid on node {}: {}. Falling back to /proc search.".format(node_name, e))
        return find_ros_node_pid(node_name)
    
    rospy.logwarn("Invalid getPid response from node {}. Falling back to /proc search.".format(node_name))
    return find_ros_node_pid(node_name)

def get_all_ros_pids():
    """
    ROS 마스터에서 등록된 모든 노드들의 PID를 가져옴 (시스템 노드는 제외).
    """
    ignore_list = ["/rosout", rospy.get_name()]
    pids = {}
    for node in rosnode.get_node_names():
        if node in ignore_list:
            continue
        pid = get_node_pid(node)
        if pid:
            pids[node] = pid
    return pids

def get_gpu_usage_by_pids(pids):
    """
    PID 리스트를 기반으로 GPU에서 실행 중인 프로세스의 GPU 사용량을 반환.
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

        # Compute 및 Graphics 프로세스 조회
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
            if proc.pid in pids.values():
                node_name = [name for name, pid in pids.items() if pid == proc.pid][0]  # PID에 해당하는 노드 찾기
                used_mem = proc.usedGpuMemory / (1024*1024)  # MiB 단위
                info = {
                    "gpu_index": i,
                    "gpu_name": dev_name.decode() if isinstance(dev_name, bytes) else dev_name,
                    "used_mem_MiB": used_mem,
                    "gpu_utilization": util.gpu,
                    "mem_utilization": util.memory
                }
                if node_name in usage_info:
                    usage_info[node_name].append(info)
                else:
                    usage_info[node_name] = [info]
    return usage_info

def main():
    rospy.init_node("ros_gpu_usage_monitor", anonymous=False)
    rate = rospy.Rate(1)  # 1Hz
    nodes_resource_pub = rospy.Publisher('/nodes_resource', String, queue_size=100)

    # NVML 초기화
    try:
        pynvml.nvmlInit()
    except pynvml.NVMLError as e:
        rospy.logerr("NVML initialization failed: {}".format(e))
        return

    rospy.loginfo("Monitoring GPU usage for all ROS processes...")

    while not rospy.is_shutdown():
        ros_pids = get_all_ros_pids()
        gpu_usage = get_gpu_usage_by_pids(ros_pids)
        output_data = {"ros_pids": ros_pids, "gpu_usage": gpu_usage}
        msg = String()
        msg.data = json.dumps(output_data)
        nodes_resource_pub.publish(msg)
        rospy.loginfo("Published GPU usage: {}".format(msg.data))
        rate.sleep()

    rospy.loginfo("Shutting down...")
    pynvml.nvmlShutdown()

if __name__ == "__main__":
    main()
