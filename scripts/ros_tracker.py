import rosgraph
import psutil

def find_topic_connections(target_topic):
    master = rosgraph.Master('/ros_topic_tracker')
    pubs, subs, _ = master.getSystemState()

    publishers = []
    subscribers = []

    # publisher 처리
    for topic, nodes in pubs:
        if topic == target_topic:
            filtered_nodes = []
            for n in nodes:
                if 'topics_hzbw' not in n:
                    filtered_nodes.append(n)
            publishers = filtered_nodes
            # 토픽은 유일하다고 가정하므로 break해도 됩니다.
            break

    # subscriber 처리
    for topic, nodes in subs:
        if topic == target_topic:
            filtered_nodes = []
            for n in nodes:
                if 'topics_hzbw' not in n:
                    filtered_nodes.append(n)
            subscribers = filtered_nodes
            break

    return publishers, subscribers

def find_node_connections(node_name):
    # node_name에 'topics_hzbw'가 포함되어 있으면 빈 리스트 반환
    if 'topics_hzbw' in node_name:
        return [], []

    master = rosgraph.Master('/ros_topic_tracker')
    pubs, subs, _ = master.getSystemState()

    pub_topics = []
    sub_topics = []

    # publisher에서 node_name과 연결된 토픽 검색 (모든 노드가 'topics_hzbw'를 포함하지 않아야 함)
    for topic, nodes in pubs:
        valid = True
        for n in nodes:
            if 'topics_hzbw' in n:
                valid = False
                break
        if node_name in nodes and valid:
            pub_topics.append(topic)

    # subscriber에서 node_name과 연결된 토픽 검색 (모든 노드가 'topics_hzbw'를 포함하지 않아야 함)
    for topic, nodes in subs:
        valid = True
        for n in nodes:
            if 'topics_hzbw' in n:
                valid = False
                break
        if node_name in nodes and valid:
            sub_topics.append(topic)

    return pub_topics, sub_topics

def find_topic_connections_with_metrics(target_topic, topic_metrics_dict):
    """
    topic_metrics_dict 예시:
      {
          '/some/topic': {'hz': 10.0, 'bw': 12345.67},
          ...
      }
    """
    master = rosgraph.Master('/ros_topic_tracker')
    pubs, subs, _ = master.getSystemState()

    publishers = []
    subscribers = []

    # publisher 처리
    for topic, nodes in pubs:
        if topic == target_topic:
            filtered_nodes = []
            for n in nodes:
                if 'topics_hzbw' not in n:
                    filtered_nodes.append(n)
            publishers = filtered_nodes
            break

    # subscriber 처리
    for topic, nodes in subs:
        if topic == target_topic:
            filtered_nodes = []
            for n in nodes:
                if 'topics_hzbw' not in n:
                    filtered_nodes.append(n)
            subscribers = filtered_nodes
            break

    # 메트릭 추출
    metrics = topic_metrics_dict.get(target_topic, {'hz': 0.0, 'bw': 0.0})

    result = {
        'topic': target_topic,
        'hz': metrics['hz'],
        'bw': metrics['bw'],
        'publishers': publishers,
        'subscribers': subscribers
    }
    return result

def find_node_pid(node_name):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # cmdline 리스트를 for-loop으로 합치기
            cmdline_list = proc.info.get('cmdline', [])
            full_cmdline = ""
            for part in cmdline_list:
                full_cmdline += part + " "
            full_cmdline = full_cmdline.strip()

            if node_name in full_cmdline:
                return proc.info['pid'], full_cmdline
        except Exception:
            continue
    return None, None
