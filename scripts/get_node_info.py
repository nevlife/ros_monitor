import yaml
import rosnode
import psutil
import rospy
import subprocess
import re

class NodeMetrics:
    def __init__(self, yaml_file=None):
        rospy.init_node("node_resource_monitor", anonymous=True)

        if yaml_file is None:
            base_path = os.path.dirname(os.path.abspath(__file__))
            self.yaml_file = os.path.join(base_path, '../cfg/node_lst.yaml')
            
        self.nodes = self.load_nodes()
        
    def load_nodes(self):
        """Load node names from YAML file"""
        try:
            with open(self.yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('nodes', [])
        except Exception as e:
            rospy.logwarn(f"Failed to load nodes from YAML file: {e}")
            return []
    
    def get_node_pid(self, node_name):
        """Get the PID of a ROS node"""
        try:
            # Retrieve node info
            node_info = rosnode.get_node_info_description(node_name)
            # Extract PID from the node info
            pid_line = [line for line in node_info.split('\n') if 'Pid:' in line]
            if pid_line:
                pid = int(pid_line[0].split('Pid:')[-1].strip())
                return pid
        except Exception as e:
            rospy.logwarn(f"Failed to get PID for node {node_name}: {e}")
            return None
    
    def get_metrics(self, pid):
        """Get CPU and memory usage of a process by PID"""
        try:
            proc = psutil.Process(pid)
            cpu_usage = proc.cpu_percent(interval=0.1) / psutil.cpu_count()
            memory_usage = proc.memory_info().rss / (1024 * 1024)  # Convert to MB
            return f"{cpu_usage:.2f}%", f"{memory_usage:.2f} MB"
        except psutil.NoSuchProcess:
            return "N/A", "N/A"
        except Exception as e:
            rospy.logwarn(f"Failed to get metrics for PID {pid}: {e}")
            return "N/A", "N/A"
    
    def monitor_nodes(self):
        """Monitor all nodes and print their metrics"""
        for node in self.nodes:
            print(f"Checking node: {node}")
            pid = self.get_node_pid(node)
            if pid:
                cpu_usage, memory_usage = self.get_metrics(pid)
                print(f"Node: {node}, PID: {pid}, CPU: {cpu_usage}, Memory: {memory_usage}")
            else:
                print(f"Node: {node}, PID: Not found")

if __name__ == "__main__":
    NodeMetrics()
