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
        self.resource_pub = rospy.Publisher(ns_join("~", name[1:], "resource"), String, queue_size=20)

    def publish(self):
        self.cpu = self.proc.cpu_percent()
        self.mem = self.proc.memory_info().rss
        
        usage = {
            'node': self.name,
            'cpu': self.cpu,
            'mem': self.mem
        }
        usage = {key: value for key, value in zip(['node', 'cpu', 'mem'], [self.name, self.cpu, self.mem])}

        print(usage)
        usage_json = json.dumps(usage)
        
        self.resource_pub.publish(String(usage_json))

    def alive(self):
        return self.proc.is_running()
    
class NodeManager:
    def __init__(self):
        self.yaml_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../cfg/node_lst.yaml')
        self.node_lst = set()
        self.load_yaml_file()
        
    def load_yaml_file(self):
        try:
            with open(self.yaml_file, 'r') as file:
                self.node_lst = file.read().splitlines()
        except Exception as e:
            rospy.logerr(f"Failed to read YAML file: {e}")
            self.node_lst = set()
    
    def monitored(self, node_name):
        return node_name in self.node_lst
        
def main():
    rospy.init_node("node_resource_monitor")
    master = rospy.get_master()

    poll_period = rospy.get_param('~poll_period', 1.0)
    #source_list = rospy.get_param('~source_list', [])
  
    this_ip = os.environ.get("ROS_IP")
    node_map = {}
    #ignored_nodes = set()

    #node_manager = NodeManager()
    #cpu_publish = rospy.Publisher("~total_cpu", Float32, queue_size=20)

    # mem_topics = ["available", "used", "free", "active", "inactive", "buffers", "cached", "shared", "slab"]

    # vm = psutil.virtual_memory()
    
    # tmp = []
    # for topic in mem_topics:
    #     if topic in dir(vm):
    #         tmp.append(topic)
    # mem_topics = tmp
    
    #mem_publishers = []
    #for mem_topic in mem_topics:
        #mem_publishers.append(rospy.Publisher("~total_%s_mem" % mem_topic, UInt64, queue_size=20))

    while not rospy.is_shutdown():
        
        valid_nodes = [
            node for node in rosnode.get_node_names()
            if node not in node_map #and node_manager.monitored(node)
            ]
        
        for node in valid_nodes: #ros 마스터에 있는 모든 노드 순회
            # if node in node_map or node in ignored_nodes:
            #     continue

            # if not node_manager.monitored(node):
            #     continue
            
            node_api = rosnode.get_api_uri(master, node, skip_cache=True)[2]

            if not node_api:
                rospy.logerr("[monitor] failed to get api of node %s (%s)" % (node, node_api))
                continue

            ros_ip = node_api[7:] # strip http://
            ros_ip = ros_ip.split(':')[0] # strip :<port>/
            local_node = "localhost" in node_api or \
                  "127.0.0.1" in node_api or \
                  (this_ip is not None and this_ip == ros_ip) or \
                  subprocess.check_output("hostname").decode('utf-8').strip() in node_api
            
            # if not local_node:
            #     ignored_nodes.add(node)
            #     rospy.loginfo("[monitor] ignoring node %s with URI %s" % (node, node_api))
            #     continue

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
                        node_map[node] = Node(name=node, pid=pid)
                    except psutil.NoSuchProcess:
                        rospy.logwarn("[monitor] psutil can't see %s (pid = %d). Ignoring" % (node, pid))
                        # ignored_nodes.add(node)
                    else:
                        rospy.loginfo("[monitor] adding new node %s" % node)

        for node_name, node in list(node_map.items()):
            if node.alive():
                node.publish()
            else:
                rospy.logwarn("[monitor] lost node %s" % node_name)
                del node_map[node_name]

        #cpu_publish.publish(Float32(psutil.cpu_percent()))

        #vm = psutil.virtual_memory()
        
        # for mem_topic, mem_publisher in zip(mem_topics, mem_publishers):
        #     mem_publisher.publish(UInt64(getattr(vm, mem_topic)))

        rospy.sleep(poll_period)

    rospy.loginfo("[monitor] shutting down")
    
if __name__ == "__main__":
    main()
    #MetricsManager()