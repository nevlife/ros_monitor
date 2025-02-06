#!/usr/bin/env python3
import os
import json
import rospy
from std_msgs.msg import Float32MultiArray, String

class JSONManager:
    def __init__(self):
        self.data = {
            'total_resource': {
                'cpu_user' : None,
                'cpu_nice' : None,
                'cpu_system' : None,
                'cpu_idle' : None,
                'cpu_iowait' : None,
                'cpu_irq' : None,
                'cpu_softirq' : None,
                'cpu_steal' : None,
                'cpu_guest' : None,
                'cpu_guest_nice' : None,
                'cpu_usage_percent' : None,
                'cpu_temp' : None,
                'cpu_load_1min' : None,
                'cpu_load_5min' : None,
                'cpu_load_15min' : None,
                
                'mem_used' : None,
                'mem_total' : None,
                'mem_usage_percent' : None,
                
                'gpu_usage_percent' : None,
                'gpu_mem_used' : None,
                'gpu_mem_total' : None,
                'gpu_mem_usage' : None,
                'gpu_temp' : None,
            },
            
            'topics_hzbw': {},
            'node_resource_usage': {},
        }
        #현재 파일 기준으로 설정
        self.file_path = os.path.join(os.path.dirname(__file__), "../data/diag.json")
        
        #check directory
        # directory = os.path.dirname(self.file_path)
        # if not os.path.exists(directory):
        #     os.makedirs(directory)
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
        
        #open json file
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as file:
                    content = file.read().strip()
                    if content:
                        self.data = json.loads(content)
                    else:
                        rospy.logwarn('json file is empty.')
            except json.JSONDecodeError as e:
                rospy.logwarn(f'failed to decode json file: {e}')
        else:
            rospy.logwarn('json file not found.')


    def save_to_file(self, event=None):
        '''save json data to file'''
        with open(self.file_path, 'w') as file:
            json.dump(self.data, file, indent=4)

    def update_total_resource(self, key, value):
        self.data['total_resource'][key] = value

    def update_topic_hzbw(self, topic_name, hz, bw):
        if 'topics_hzbw' not in self.data:
            self.data['topics_hzbw'] = {}
        self.data['topics_hzbw'][topic_name] = {'hz': hz, 'bw': bw}

    def update_node_resource(self, node_name, cpu, mem):
        if 'node_resource_usage' not in self.data:
            self.data['node_resource_usage'] = {}
        self.data['node_resource_usage'][node_name] = {'cpu': cpu, 'mem': mem}


manager = JSONManager()

def total_resource_callback(total_resource_sub):
    keys = [
        'cpu_user', 'cpu_nice', 'cpu_system', 'cpu_idle', 'cpu_iowait',
        'cpu_irq', 'cpu_softirq', 'cpu_steal', 'cpu_guest', 'cpu_guest_nice',
        'cpu_usage_percent', 'cpu_temp', 'cpu_load_1min', 'cpu_load_5min', 'cpu_load_15min',
        'mem_used', 'mem_total', 'mem_usage_percent',
        'gpu_usage_percent', 'gpu_mem_used', 'gpu_mem_total', 'gpu_mem_usage', 'gpu_temp'
    ]

    try:
        data = total_resource_sub.data

        for i, key in enumerate(keys):
            manager.update_total_resource(key, data[i])
            
    except IndexError:
        rospy.logwarn('Invalid total_resource data')

def topics_hzbw_callback(topics_hzbw_sub):
    try:
        data = json.loads(topics_hzbw_sub.data)
        
        #current_topics = set()
        # for item in data:
        #     topic_name = item.get('topic', 'unknown')
        #     hz = item.get('hz')
        #     bw = item.get('bw')
            
        #     manager.update_topic_hzbw(topic_name, hz, bw)
            
        #     current_topics.add(topic_name)
        
        # existing_topics = set(manager.data['topics_hzbw'].keys())
        
        # disconnected_topics = existing_topics - current_topics
        
        # for topic in disconnected_topics:
        #     del manager.data['topics_hzbw'][topic]
        
        for topic in data:
            #print(topic)
            manager.update_topic_hzbw(
                topic_name=topic.get('topic'),
                hz=topic.get('hz'),
                bw=topic.get('bw'),
            )
            
    except json.JSONDecodeError as e:
        rospy.logwarn(f'failed to parse topics_hzbw json: {e}')

def nodes_resource_callback(nodes_resource_sub):
    try:
        data = json.loads(nodes_resource_sub.data)
        #print(type(data))
        for node in data:
            manager.update_node_resource(
                node_name=node.get('node'),
                cpu=node.get('cpu'),
                mem=node.get('mem'),
            )
            
    except json.JSONDecodeError as e:
        rospy.logwarn(f'failed to parse nodes_resource json: {e}')

def main():
    rospy.init_node('diag_listener', anonymous=True)

    #save 1 sec
    rospy.Timer(rospy.Duration(1), manager.save_to_file)

    total_resource_sub = rospy.Subscriber('/total_resource', Float32MultiArray, total_resource_callback)
    topic_hzbw_sub = rospy.Subscriber('/topics_hzbw', String, topics_hzbw_callback)
    nodes_resource_sub = rospy.Subscriber('/node_resource_usage', String, nodes_resource_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
