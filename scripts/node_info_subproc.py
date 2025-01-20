#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import subprocess
import threading


class ROSTopicMetrics:
    def __init__(self, topic):
        self.topic = topic
        self.bandwidth = None  # Store the latest bandwidth value
        self.thread = threading.Thread(target=self.monitor_bw)
        self.thread.daemon = True  # Automatically terminates when the main program exits
        self.thread.start()  # Start monitoring immediately

    def monitor_bw(self):
        """Run rostopic bw and parse its output."""
        command = ["rostopic", "bw", self.topic]
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            for line in process.stdout:
                if "average:" in line:
                    # Parse bandwidth information from the output
                    parts = line.split()
                    self.bandwidth = float(parts[1])  # Extract bandwidth value
                    unit = parts[2]  # e.g., "bytes/sec"
                    rospy.loginfo(f"[{self.topic}] Bandwidth: {self.bandwidth} {unit}")
        except Exception as e:
            rospy.logerr(f"Failed to run rostopic bw: {e}")

    def get_metrics(self):
        """Return metrics (Bandwidth)."""
        return {"hz": "N/A", "bandwidth": self.bandwidth}


class MetricsManager:
    def __init__(self, topics):
        self.monitors = {}
        self.topics = topics
        self.initialize_monitors()

    def initialize_monitors(self):
        rospy.loginfo(f"Initializing monitors for {len(self.topics)} topics...")
        for topic in self.topics:
            self.monitors[topic] = ROSTopicMetrics(topic)

    def get_metrics(self):
        results = {}
        for topic, monitor in self.monitors.items():
            results[topic] = monitor.get_metrics()
        return results


if __name__ == "__main__":
    rospy.init_node("bandwidth_monitor")
    topics = ["/example_topic1", "/example_topic2"]  # Add your topics here

    manager = MetricsManager(topics)
    rate = rospy.Rate(1)  # 1 Hz

    try:
        while not rospy.is_shutdown():
            metrics = manager.get_metrics()
            for topic, data in metrics.items():
                rospy.loginfo(f"{topic}: Hz = {data['hz']}, Bandwidth = {data['bandwidth']} bytes/sec")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
