#!/usr/bin/python3
# -*- coding: utf-8 -*-
import functools
import os
import subprocess
import json
import sys

import rosnode
import rospy

import psutil
import pynvml

try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

from std_msgs.msg import String

# core.py 모듈 import를 위한 경로 설정
scripts_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(scripts_dir)

# core.py에서 Monitor 클래스 import
from core.core import Monitor, AggregationStrategies

def ns_join(*names):
    return functools.reduce(rospy.names.ns_join, names, "")

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
        
        print(data)
        return data

    def alive(self):
        return self.proc.is_running()

class GpuNodeManager:
    def __init__(self,):
        try:
            pynvml.nvmlInit()
        except pynvml.NVMLError as e:
            rospy.logerr("NVML initialization failed: {}".format(e))
        
        
def main():
    rospy.init_node('nodes_resource_node', anonymous=False)
    master = rospy.get_master()
    rate = rospy.Rate(1)

    poll_period = rospy.get_param('~poll_period', 1.0)
    node_map = {}
    
    # 기존 String 메시지 발행자
    #nodes_resource_pub = rospy.Publisher('/nodes_resource', String, queue_size=100)
    
    # Monitor 객체 생성 - '/monitoring' 토픽으로 발행
    monitor = Monitor(monitorDescription="Node Resource Monitor", monitoring_type="nodes_resource")

    while not rospy.is_shutdown():
        
        valid_nodes = [
            node 
            for node in rosnode.get_node_names()
            if node not in node_map
        ]

        for node in valid_nodes:
            try:
                node_api = rosnode.get_api_uri(master, node, skip_cache=True)[2]

                if not node_api:
                    rospy.logerr("[monitor] failed to get api of node %s (%s)" % (node, node_api))
                    continue
                
                try:
                    resp = ServerProxy(node_api).getPid('/NODEINFO')
                except:
                    rospy.logerr("[monitor] failed to get pid of node %s (api is %s)" % (node, node_api))
                else:
                    try:
                        pid = resp[2]
                    except:
                        rospy.logerr("[monitor] failed to get pid for node %s from NODEINFO response: %s" % (node, resp))
                    else:
                        try:
                            node_map[node] = NodeManager(name=node, pid=pid)

                        except psutil.NoSuchProcess:
                            rospy.logwarn("[monitor] psutil can't see %s (pid = %d). Ignoring" % (node, pid))
                        else:
                            rospy.loginfo("[monitor] adding new node %s" % node)
                        
            except (IndexError, psutil.NoSuchProcess):
                rospy.logwarn(f"[monitor] Failed to get PID for node {node}")
                
            except Exception as e:
                rospy.logerr(f"[monitor] Unexpected error for node {node}: {e}")
                
        nodes_data = [
            node_manager.get_metrics()
            for node_manager in sorted(node_map.values(), key=lambda nm: nm.name)
            if node_manager.alive()
        ]

        # Monitoring 추가: 각 노드의 CPU 및 메모리 사용량을 Monitor 객체에 추가
        for node_data in nodes_data:
            node_name = node_data['node']
            cpu_usage = node_data['cpu']
            mem_usage = node_data['mem'] / (1024 * 1024)  # 바이트를 MB로 변환
            
            # 노드 이름의 슬래시를 언더스코어로 변환
            #node_name_safe = node_name.replace('/', '_').lstrip('_')
            
            # CPU 사용량 모니터링: 경고 수준 설정 (80% 이상은 경고, 90% 이상은 심각)
            error_level_cpu = 0.0
            if cpu_usage > 90:
                error_level_cpu = 2.0
            elif cpu_usage > 80:
                error_level_cpu = 1.0
            tmp_target = -1.0
            # 메모리 모니터링 (여기서는 단순히 기록)
            monitor.addValue(f"cpu_{node_name}", cpu_usage, "%", tmp_target, error_level_cpu)
            monitor.addValue(f"mem_{node_name}", mem_usage, "MB", tmp_target, 0.0)

        node_map = {node_name: node_manager for node_name, node_manager in node_map.items() if node_manager.alive()}

        # 기존 발행자에도 데이터 전송
        #nodes_resource_pub.publish(json.dumps(nodes_data))
        
        rospy.sleep(poll_period)

    rospy.loginfo("[monitor] shutting down")


if __name__ == "__main__":
    main()