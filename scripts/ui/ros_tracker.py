import rosgraph
import psutil
def find_topic_connections(target_topic):
    master = rosgraph.Master('/ros_topic_tracker')
    pubs, subs, _ = master.getSystemState()

    publishers = []
    subscribers = []

    for topic, nodes in pubs:
        if topic == target_topic:
            publishers = [n for n in nodes if 'topics_hzbw' not in n]

    for topic, nodes in subs:
        if topic == target_topic:
            subscribers = [n for n in nodes if 'topics_hzbw' not in n]

    return publishers, subscribers


def find_node_connections(node_name):
    if 'topics_hzbw' in node_name:
        return [], []

    master = rosgraph.Master('/ros_topic_tracker')
    pubs, subs, _ = master.getSystemState()

    pub_topics = []
    sub_topics = []

    for topic, nodes in pubs:
        if node_name in nodes and all('topics_hzbw' not in n for n in nodes):
            pub_topics.append(topic)

    for topic, nodes in subs:
        if node_name in nodes and all('topics_hzbw' not in n for n in nodes):
            sub_topics.append(topic)

    return pub_topics, sub_topics

def find_topic_connections_with_metrics(target_topic, topic_metrics_dict):
    """
    topic_metrics_dict: {
        '/some/topic': {'hz': 10.0, 'bw': 12345.67},
        ...
    }
    """
    master = rosgraph.Master('/ros_topic_tracker')
    pubs, subs, _ = master.getSystemState()

    publishers = []
    subscribers = []

    for topic, nodes in pubs:
        if topic == target_topic:
            publishers = [n for n in nodes if 'topics_hzbw' not in n]

    for topic, nodes in subs:
        if topic == target_topic:
            subscribers = [n for n in nodes if 'topics_hzbw' not in n]

    # 메트릭 추출
    metrics = topic_metrics_dict.get(target_topic, {'hz': 0.0, 'bw': 0.0})

    return {
        'topic': target_topic,
        'hz': metrics['hz'],
        'bw': metrics['bw'],
        'publishers': publishers,
        'subscribers': subscribers
    }


def find_node_pid(node_name):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if node_name in ' '.join(proc.info['cmdline']):
                return proc.info['pid'], ' '.join(proc.info['cmdline'])
        except Exception:
            continue
    return None, None