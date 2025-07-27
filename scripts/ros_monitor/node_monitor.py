#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
import re

class NodeResourceMonitor(Node):
    def __init__(self):
        super().__init__('node_resource_monitor')

        self.publisher_ = self.create_publisher(String, 'node_resources', 10)
        self.timer = self.create_timer(1.0, self.monitor_callback)

    def monitor_callback(self):
        node_resources = {}

        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''

                # ROS2 노드 찾기
                if '__node:=' in cmdline:
                    node_match = re.search(r'__node:=([^\s]+)', cmdline)
                    if node_match:
                        node_name = node_match.group(1)

                        # 자원 정보 수집
                        node_resources[node_name] = {
                            'pid': proc.info['pid'],
                            'cpu_percent': proc.cpu_percent(interval=0.1),
                            'memory_mb': proc.memory_info().rss / 1024 / 1024,
                            'memory_percent': proc.memory_percent(),
                            'num_threads': proc.num_threads()
                        }

            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        # 퍼블리시
        msg = String()
        msg.data = json.dumps(node_resources, indent=2)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    monitor = NodeResourceMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()