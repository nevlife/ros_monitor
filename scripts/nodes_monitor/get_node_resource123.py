#!/usr/bin/env python3

import functools
import os
import subprocess
import json
import rosnode
import rospy
import psutil

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from std_msgs.msg import String


def ns_join(*names):
    return functools.reduce(rospy.names.ns_join, names, "")


class Node:
    def __init__(self, name, pid):
        self.name = name
        self.proc = psutil.Process(pid)

    def publish(self):
        self.cpu = self.proc.cpu_percent()
        self.mem = self.proc.memory_info().rss

        usage = {
            'node': self.name,
            'cpu': self.cpu,
            'mem': self.mem
        }

        return usage

    def alive(self):
        return self.proc.is_running()


def main():
    rospy.init_node("node_resource_monitor")
    master = rospy.get_master()

    poll_period = rospy.get_param('~poll_period', 1.0)
    this_ip = os.environ.get("ROS_IP")
    node_map = {}

    # JSON 형식으로 CPU/RAM 사용량을 발행할 퍼블리셔 생성
    node_resource_pub = rospy.Publisher("/node_resource_usage", String, queue_size=10)

    while not rospy.is_shutdown():
        valid_nodes = [
            node for node in rosnode.get_node_names()
            if node not in node_map
        ]

        for node in valid_nodes:
            node_api = rosnode.get_api_uri(master, node, skip_cache=True)[2]

            if not node_api:
                rospy.logerr("[monitor] failed to get API URI of node %s (%s)", node, node_api)
                continue

            try:
                resp = ServerProxy(node_api).getPid('/NODEINFO')
                pid = resp[2]
                node_map[node] = Node(name=node, pid=pid)
                rospy.loginfo("[monitor] adding new node %s", node)

            except Exception as e:
                rospy.logerr("[monitor] failed to get PID for node %s: %s", node, str(e))

        all_nodes_data = {}  # 모든 노드의 데이터를 JSON으로 저장할 딕셔너리

        for node_name, node in list(node_map.items()):
            if node.alive():
                data = node.publish()
                all_nodes_data[node_name] = data  # 각 노드의 데이터를 JSON 객체에 추가
            else:
                rospy.logwarn("[monitor] lost node %s", node_name)
                del node_map[node_name]

        # 모든 노드 데이터를 JSON 문자열로 변환 후 발행
        json_message = json.dumps(all_nodes_data)
        node_resource_pub.publish(String(json_message))

        rospy.sleep(poll_period)

    rospy.loginfo("[monitor] shutting down")


if __name__ == "__main__":
    main()
