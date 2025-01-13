import os
import inspect

import rospy
import rosgraph
import rostopic 
import ros_topic
import yaml
from pathlib import Path


def main():
    pth = os.getcwd()
    parent_pth = Path(pth).parent
    print(parent_pth)
    cfg = os.path.join(str(parent_pth) + '/cfg/topic_lst.yaml')

    #get topic name list
    with open(cfg) as f:
        topic_lst = yaml.load(f, Loader=yaml.FullLoader)
        topic_lst = topic_lst.split(' ')
        print(topic_lst)

    topic_lst = get_topic_lst()

    for topic in topic_lst:
        rospy.init_node('chosen_node_name')  
        hz = ros_topic.ROSTopicHz(-1)  
        sub_topic = rospy.Subscriber(topic, rospy.AnyMsg, hz.callback_hz, callback_args=topic)  
        rospy.sleep(1)  
        # print(topic)
        hz.print_hz([topic])
        b = ros_topic.ROSTopicBandwidth()  
        s2 = rospy.Subscriber(topic, rospy.AnyMsg, b.callback)  
        rospy.sleep(1)  
        b.print_bw()
        
def get_topic_lst():
    master = rosgraph.Master('/rostopic')
    pubs, subs = rostopic.get_topic_list(master=master)
    topic_data = {}

    print(f"subs {len(subs)}")
    for topic in pubs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
        topic_data[name]['publishers'] = topic[2]
    print(f"subs {len(pubs)}")
    
    for topic in subs:
        name = topic[0]
        if name not in topic_data:
            topic_data[name] = {}
            topic_data[name]['type'] = topic[1]
        topic_data[name]['subscribers'] = topic[2]
    topic_lst = sorted(topic_data.keys())
    
    #필요없는 거거
    topic_lst.remove('/clock')
    topic_lst.remove('/rosout')
    topic_lst.remove('/rosout_agg')
    

    print(sorted(topic_data.keys()))
    return sorted(topic_data.keys())


if __name__ == '__main__':
    main():