#!/usr/bin/env python3

import rospy
from rospy import get_published_topics

from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame, QPushButton, QHBoxLayout, QTableWidget, QTableWidgetItem, QHeaderView
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from std_msgs.msg import Float32MultiArray, String
from message_filters import ApproximateTimeSynchronizer, Subscriber
import json

class SystemMonitorGUI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.sys_status_data = {
            'CPU Usage': 'N/A',
            'CPU Freq': 'N/A',
            'GPU Usage': 'N/A',
            'Memory': 'N/A',
        }

        self.init_components()
        self.init_layout()
        self.init_msgs_sync()

    def init_components(self):
        # System status labels
        self.sys_status_labels = {}
        for key in self.sys_status_data:
            label = QLabel(f'{key}: {self.sys_status_data[key]}')
            font = label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
            self.sys_status_labels[key] = label

        # Buttons
        self.button_1 = QPushButton('Start')
        self.button_2 = QPushButton('Stop')
        self.button_3 = QPushButton('Reset')
        self.button_1.clicked.connect(self.start_clicked)
        self.button_2.clicked.connect(self.stop_clicked)
        self.button_3.clicked.connect(self.reset_clicked)

        # Node info table
        topic_hzbw_lst = ['topic names', 'hz', 'bandwidth']
        self.topic_hzbw_table = QTableWidget()
        self.topic_hzbw_table.setColumnCount(len(topic_hzbw_lst))
        self.topic_hzbw_table.setHorizontalHeaderLabels(topic_hzbw_lst)
        
        self.topic_hzbw_table.setColumnWidth(0, 100)  # Node Names column
        self.topic_hzbw_table.setColumnWidth(1, 100)  # Hz column
        self.topic_hzbw_table.setColumnWidth(2, 120)  # Bandwidth column

        node_resource_lst = ['topic names', 'cpu usage', 'ram usage']
        self.node_resource_table = QTableWidget()
        self.node_resource_table.setColumnCount(len(node_resource_lst))
        self.node_resource_table.setHorizontalHeaderLabels(node_resource_lst)
        
        self.node_resource_table.setColumnWidth(0, 100)  # Node Names column
        self.node_resource_table.setColumnWidth(1, 100)  # Hz column
        self.node_resource_table.setColumnWidth(2, 120)  # Bandwidth column
        
    def init_layout(self):
        '''Set up the GUI layout'''
        main_layout = QVBoxLayout(self)

        # Buttons layout
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_1)
        button_layout.addWidget(self.button_2)
        button_layout.addWidget(self.button_3)
        button_layout.setSpacing(10)

        button_container = QFrame()
        button_container.setFixedHeight(50)
        button_layout.setAlignment(Qt.AlignLeft)
        button_container.setLayout(button_layout)

        # System info layout
        sys_layout = QGridLayout()
        for row, label in enumerate(self.sys_status_labels.values()):
            sys_layout.addWidget(label, row, 0)

        sys_container = QFrame()
        sys_container.setLayout(sys_layout)
        sys_container.setFrameShape(QFrame.Box)
        sys_container.setLineWidth(2)

        #topic_hzbw layout
        topic_hzbw_layout = QVBoxLayout()
        topic_hzbw_layout.addWidget(self.topic_hzbw_table)

        topic_hzbw_container = QFrame()
        topic_hzbw_container.setLayout(topic_hzbw_layout)
        topic_hzbw_container.setFrameShape(QFrame.Box)
        topic_hzbw_container.setLineWidth(2)
        self.topic_hzbw_table.horizontalHeader().setStretchLastSection(True)
        self.topic_hzbw_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        #node resource layout
        node_resource_layout = QVBoxLayout()
        node_resource_layout.addWidget(self.node_resource_table)

        node_resource_container = QFrame()
        node_resource_container.setLayout(node_resource_layout)
        node_resource_container.setFrameShape(QFrame.Box)
        node_resource_container.setLineWidth(2)
        self.node_resource_table.horizontalHeader().setStretchLastSection(True)
        self.node_resource_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # Main layout
        main_grid_layout = QGridLayout()
        main_grid_layout.addWidget(sys_container, 0, 0, 1, 1)
        main_grid_layout.addWidget(topic_hzbw_container, 0, 1, 2, 4)
        main_grid_layout.addWidget(node_resource_container, 3, 1, 2, 4)

        grid_container = QFrame()
        grid_container.setLayout(main_grid_layout)

        main_layout.addWidget(button_container)
        main_layout.addWidget(grid_container)

    def main_callback(self, sys_status_sub, topic_hzbw_sub, *node_resource_sub):
        self.sys_status_callback(sys_status_sub)
        self.topic_hzbw_callback(topic_hzbw_sub)
        self.node_resource_callback(node_resource_sub)
        
    def sys_status_callback(self, sys_status_sub):
        try:
            self.sys_status_data['CPU Usage'] = f'{sys_status_sub.data[0]:.2f}%'
            self.sys_status_data['CPU Freq'] = f'{sys_status_sub.data[1]:.2f} GHz'
            self.sys_status_data['GPU Usage'] = f'{sys_status_sub.data[2]:.2f}%'
            self.sys_status_data['Memory'] = f'{sys_status_sub.data[3]:.2f}%'
            for key, val in self.sys_status_data.items():
                self.sys_status_labels[key].setText(f'{key}: {val}')
        except IndexError as e:
            rospy.logerr(f"Invalid system status data: {e}")

    def topic_hzbw_callback(self, topic_hzbw_sub):
        try:
            topic_data = json.loads(topic_hzbw_sub.data)
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse topic_hzbw data: {e}")
            return

        # Update node info table
        self.topic_hzbw_table.setRowCount(len(topic_data))

        for row, topic in enumerate(topic_data):
            topic_name = topic["topic"]
            hz = topic["hz"]
            bw = topic["bw"]

            topic_item = QTableWidgetItem(topic_name)
            hz_item = QTableWidgetItem(f'{hz:.2f} Hz')
            bw_item = QTableWidgetItem(f'{bw:.2f} bytes/sec')

            self.topic_hzbw_table.setItem(row, 0, topic_item)
            self.topic_hzbw_table.setItem(row, 1, hz_item)
            self.topic_hzbw_table.setItem(row, 2, bw_item)
        
    def node_resource_callback(self, node_resource_sub):
        self.node_resource_table.setRowCount(len(node_resource_sub))
        for row, resource_msg in enumerate(node_resource_sub):
            try:
                resource_data = json.loads(resource_msg.data)  # JSON 파싱
                node_name = resource_data.get("node", "Unknown")
                
                cpu = resource_data.get("cpu", 0)
                ram = resource_data.get("mem", 0)

                node_item = QTableWidgetItem(node_name)
                cpu_item = QTableWidgetItem(f'{cpu:.2f}%')
                ram_item = QTableWidgetItem(f'{ram} byte')

                self.node_resource_table.setItem(row, 0, node_item)
                self.node_resource_table.setItem(row, 1, cpu_item)
                self.node_resource_table.setItem(row, 2, ram_item)
            except json.JSONDecodeError as e:
                rospy.logerr(f"Failed to parse resource monitor data: {e}")
    # def sys_status_callback(self, sys_status_sub, topic_hzbw_sub, node_resource_sub):
    #     '''Update system info and node info table'''
    #     # Update system info
    #     self.sys_status_data['CPU Usage'] = f'{sys_status_sub.data[0]:.2f}%'
    #     self.sys_status_data['CPU Freq'] = f'{sys_status_sub.data[1]:.2f} GHz'
    #     self.sys_status_data['GPU Usage'] = f'{sys_status_sub.data[2]:.2f}%'
    #     self.sys_status_data['Memory'] = f'{sys_status_sub.data[3]:.2f}%'
    #     for key, val in self.sys_status_data.items():
    #         self.sys_status_labels[key].setText(f'{key}: {val}')

    #     # Parse topic_hzbw_sub JSON data
    #     try:
    #         topic_data = json.loads(topic_hzbw_sub.data)
    #     except json.JSONDecodeError as e:
    #         rospy.logerr(f"Failed to parse topic_hzbw data: {e}")
    #         return
    #     print(topic_data)
    #     # Update node info table
    #     self.topic_hzbw_table.setRowCount(len(topic_data))

    #     for row, topic in enumerate(topic_data):
    #         topic_name = topic["topic"]
    #         hz = topic["hz"]
    #         bw = topic["bw"]

    #         topic_item = QTableWidgetItem(topic_name)
    #         hz_item = QTableWidgetItem(f'{hz:.2f} Hz')
    #         bw_item = QTableWidgetItem(f'{bw:.2f} bytes/sec')

    #         self.topic_hzbw_table.setItem(row, 0, topic_item)
    #         self.topic_hzbw_table.setItem(row, 1, hz_item)
    #         self.topic_hzbw_table.setItem(row, 2, bw_item)

    def start_clicked(self):
        rospy.loginfo('Start clicked')

    def stop_clicked(self):
        rospy.loginfo('Stop clicked')

    def reset_clicked(self):
        rospy.loginfo('Reset clicked')
        rospy.loginfo('Refreshing data...')

    def init_msgs_sync(self):
        published_topics = rospy.get_published_topics()
        resource_monitor_topics = [
            topic for topic, topic_type in published_topics if topic.startswith('/node_resource_monitor')
        ]
        
        self.subscribers = []
        self.ats_callbacks = []
        
        for topic in resource_monitor_topics:
            sub = Subscriber(topic, String)
            self.subscribers.append(sub)
            
        self.sys_status_sub = Subscriber('/get_sys_status', Float32MultiArray)  # Total sys info
        self.topic_hzbw_sub = Subscriber('/topic_hzbw', String)

        ats = ApproximateTimeSynchronizer([self.sys_status_sub, self.topic_hzbw_sub] + self.subscribers, queue_size=20, slop=1, allow_headerless=True)
        ats.registerCallback(self.main_callback)

        
# Ensure this is run in a ROS node context
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    rospy.init_node('system_monitor_gui', anonymous=True)
    app = QApplication(sys.argv)
    gui = SystemMonitorGUI()
    gui.show()
    sys.exit(app.exec_())
