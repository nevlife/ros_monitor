#!/usr/bin/python3
# -*- coding: utf-8 -*-
import functools
import os
import subprocess
import json
import re

import rosnode
import rospy

import psutil
import pynvml

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from std_msgs.msg import String

def ns_join(*names):
    return functools.reduce(rospy.names.ns_join, names, "")

def find_ros_node_pid(node_name):
    """ /proc에서 cmdline을 검색하여 노드 이름이 포함된 프로세스의 PID를 반환 """
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
    rosnode.get_api_uri()를 통해 노드의 API URI를 얻고 XMLRPC로 getPid()를 호출해 PID를 가져옴.
    실패하면 /proc에서 직접 검색합니다.
    """
    try:
        # caller_id와 대상 노드 이름을 함께 전달
        node_uri = rosnode.get_api_uri(rospy.get_name(), node_name)
    except Exception as e:
        rospy.logwarn("Failed to lookup node URI for %s: %s", node_name, str(e))
        return find_ros_node_pid(node_name)
    
    if not node_uri:
        rospy.logwarn("Empty node URI for %s", node_name)
        return find_ros_node_pid(node_name)
    
    # URI 형식 예: "http://hostname:port/"
    m = re.match(r"http://([^:]+):(\d+)/", node_uri)
    if not m:
        rospy.logwarn("Failed to parse node URI for %s: %s", node_name, node_uri)
        return find_ros_node_pid(node_name)
    host, port_str = m.group(1), m.group(2)
    try:
        port = int(port_str)
    except Exception:
        rospy.logwarn("Invalid port in node URI for %s: %s", node_name, node_uri)
        return find_ros_node_pid(node_name)
    
    try:
        client = ServerProxy("http://{}:{}/".format(host, port))
        result = client.getPid(rospy.get_name())
        # result는 보통 [statusCode, statusMessage, pid]
        if isinstance(result, list) and len(result) >= 3:
            return int(result[2])
    except Exception as e:
        rospy.logwarn("Failed to call getPid on node %s: %s", node_name, str(e))
        return find_ros_node_pid(node_name)
    
    rospy.logwarn("Invalid getPid response from node %s", node_name)
    return find_ros_node_pid(node_name)

class NodeManager:
    def __init__(self, name, pid):
        self.name = name
        self.proc = psutil.Process(pid)

    def get_metrics(self):
        data = {
            'node': self.name,
            'cpu': self.proc.cpu_percent(),
            'mem': self.proc.memory_info().rss
        }
        rospy.loginfo("%s", data)
        return data

    def alive(self):
        return self.proc.is_running()

def main():
    rospy.init_node("nodes_resource")
    master = rospy.get_master()

    poll_period = rospy.get_param('~poll_period', 1.0)
    node_map = {}
    
    nodes_resource_pub = rospy.Publisher('/nodes_resource', String, queue_size=100)

    while not rospy.is_shutdown():
        # 노드 이름 목록 중, 아직 node_map에 없는 노드들만 valid_nodes로 처리
        valid_nodes = [node for node in rosnode.get_node_names() if node not in node_map]

        for node in valid_nodes:
            try:
                pid = get_node_pid(node)
                if pid:
                    node_map[node] = NodeManager(name=node, pid=pid)
                    rospy.loginfo("[monitor] adding new node %s (pid %d)" % (node, pid))
                else:
                    rospy.logwarn("[monitor] Failed to get PID for node %s" % node)
            except Exception as e:
                rospy.logerr("[monitor] Unexpected error for node %s: %s" % (node, str(e)))
        
        # 수집된 노드들의 리소스 정보 가져오기
        nodes_data = [nm.get_metrics() for nm in sorted(node_map.values(), key=lambda nm: nm.name) if nm.alive()]
        
        # 살아있는 노드만 node_map에 유지
        node_map = {node_name: nm for node_name, nm in node_map.items() if nm.alive()}

        nodes_resource_pub.publish(json.dumps(nodes_data))
        rospy.sleep(poll_period)

    rospy.loginfo("[monitor] shutting down")

if __name__ == "__main__":
    main()
