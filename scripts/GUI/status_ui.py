#!/usr/bin/env python3

import rospy
from rospy import get_published_topics

from PySide6.QtWidgets import (
    QWidget, 
    QVBoxLayout, 
    QGridLayout,
    QLabel,
    QFrame,
    QPushButton,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QApplication
    )
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from std_msgs.msg import Float32MultiArray, String
from message_filters import ApproximateTimeSynchronizer, Subscriber
import json

def signal_handler(sig, frame):
    QApplication.quit()
    sys.exit(0)
    
class SystemMonitorGUI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.subscribers = {}  # 각 토픽의 Subscriber 객체를 저장
        self.node_resource_topics = []
        
        self.total_resource_data = {
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
        self.total_resource_labels = {}
        for key in self.total_resource_data:
            label = QLabel(f'{key}: {self.total_resource_data[key]}')
            font = label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
            self.total_resource_labels[key] = label

        # Buttons
        self.button_1 = QPushButton('Start')
        self.button_2 = QPushButton('Stop')
        self.button_3 = QPushButton('Reset')
        self.button_1.clicked.connect(self.start_clicked)
        self.button_2.clicked.connect(self.stop_clicked)
        self.button_3.clicked.connect(self.reset_clicked)

        #topic table
        topic_hzbw_lst = ['topic names', 'hz', 'bandwidth']
        self.topic_hzbw_table = QTableWidget()
        self.topic_hzbw_table.setColumnCount(len(topic_hzbw_lst))
        self.topic_hzbw_table.setHorizontalHeaderLabels(topic_hzbw_lst)
        self.topic_hzbw_table.verticalHeader().setVisible(False)   
        
        self.topic_hzbw_table.setColumnWidth(0, 100)  # Node Names column
        self.topic_hzbw_table.setColumnWidth(1, 100)  # Hz column
        self.topic_hzbw_table.setColumnWidth(2, 120)  # Bandwidth column

        #node table
        node_resource_lst = ['node names', 'cpu usage', 'ram usage']
        self.node_resource_table = QTableWidget()
        self.node_resource_table.setColumnCount(len(node_resource_lst))
        self.node_resource_table.setHorizontalHeaderLabels(node_resource_lst)
        self.node_resource_table.verticalHeader().setVisible(False)        
        
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
        for row, label in enumerate(self.total_resource_labels.values()):
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

    def main_callback(self, total_resource_sub, topic_hzbw_sub, *node_resource_sub):
        self.total_resource_callback(total_resource_sub)
        self.topic_hzbw_callback(topic_hzbw_sub)
        self.node_resource_callback(node_resource_sub)
        
    def total_resource_callback(self, total_resource_sub):
        try:
            self.total_resource_data['CPU Usage'] = f'{total_resource_sub.data[0]:.2f}%'
            self.total_resource_data['CPU Freq'] = f'{total_resource_sub.data[1]:.2f} GHz'
            self.total_resource_data['GPU Usage'] = f'{total_resource_sub.data[2]:.2f}%'
            self.total_resource_data['Memory'] = f'{total_resource_sub.data[3]:.2f}%'
        except (IndexError, AttributeError) as e:
            rospy.logwarn(f"Failed to process system status: {e}")
            self.total_resource_data['CPU Usage'] = 'N/A'
            self.total_resource_data['CPU Freq'] = 'N/A'
            self.total_resource_data['GPU Usage'] = 'N/A'
            self.total_resource_data['Memory'] = 'N/A'

        for key, val in self.total_resource_data.items():
            self.total_resource_labels[key].setText(f'{key}: {val}')

    def topic_hzbw_callback(self, topic_hzbw_sub):
        try:
            topic_data = json.loads(topic_hzbw_sub.data)
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to parse topic_hzbw data: {e}")
            return

        # 테이블 업데이트
        self.topic_hzbw_table.setRowCount(len(topic_data))
        
        for row, topic in enumerate(topic_data):
            try:
                topic_name = topic["topic"]
                hz = topic["hz"]
                bw = topic["bw"]

                topic_item = QTableWidgetItem(topic_name)
                hz_item = QTableWidgetItem(f'{hz:.2f} Hz')
                bw_item = QTableWidgetItem(f'{bw:.2f} bytes/sec')

                self.topic_hzbw_table.setItem(row, 0, topic_item)
                self.topic_hzbw_table.setItem(row, 1, hz_item)
                self.topic_hzbw_table.setItem(row, 2, bw_item)
            except KeyError as e:
                rospy.logwarn(f"Missing data in topic_hzbw entry: {e}")
                
    def node_resource_callback(self, node_resource_sub):
        try:
            resource_data = json.loads(node_resource_sub.data)  # JSON 파싱
            node_name = resource_data.get("node", "Unknown")
            
            if not node_name.startswith("/node_resource_monitor"):
                #rospy.loginfo(f"Skipping node {node_name}, does not match '/node_resource_monitor'")
                return
            
            cpu = resource_data.get("cpu", 0)
            ram = resource_data.get("mem", 0)
            print(cpu)
            # 기존 행 업데이트 또는 새 행 추가
            row_count = self.node_resource_table.rowCount()
            for row in range(row_count):
                if self.node_resource_table.item(row, 0).text() == node_name:
                    self.node_resource_table.setItem(row, 1, QTableWidgetItem(f'{cpu:.2f}%'))
                    self.node_resource_table.setItem(row, 2, QTableWidgetItem(f'{ram} byte'))
                    return

            # 새로운 노드 데이터 추가
            row = self.node_resource_table.rowCount()
            self.node_resource_table.insertRow(row)
            self.node_resource_table.setItem(row, 0, QTableWidgetItem(node_name))
            self.node_resource_table.setItem(row, 1, QTableWidgetItem(f'{cpu:.2f}%'))
            self.node_resource_table.setItem(row, 2, QTableWidgetItem(f'{ram} byte'))
            
            rospy.loginfo(f'new nofe added : {node_name}')
            self.add_new_node_to_ui(node_name)
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to parse resource monitor data: {e}")

    # def total_resource_callback(self, total_resource_sub, topic_hzbw_sub, node_resource_sub):
    #     '''Update system info and node info table'''
    #     # Update system info
    #     self.total_resource_data['CPU Usage'] = f'{total_resource_sub.data[0]:.2f}%'
    #     self.total_resource_data['CPU Freq'] = f'{total_resource_sub.data[1]:.2f} GHz'
    #     self.total_resource_data['GPU Usage'] = f'{total_resource_sub.data[2]:.2f}%'
    #     self.total_resource_data['Memory'] = f'{total_resource_sub.data[3]:.2f}%'
    #     for key, val in self.total_resource_data.items():
    #         self.total_resource_labels[key].setText(f'{key}: {val}')

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

    def init_msgs_sync(self):
        self.total_resource_sub = rospy.Subscriber('/total_resource', Float32MultiArray, self.total_resource_callback)  # Total sys info
        self.topic_hzbw_sub = rospy.Subscriber('/topic_hzbw', String, self.topic_hzbw_callback)
        
        published_topics = rospy.get_published_topics()
        resource_monitor_topics = [
            rospy.Subscriber(topic, String, self.node_resource_callback)
            for topic, topic_type in published_topics
            if topic.startswith('/node_resource_monitor')
            ]

# Ensure this is run in a ROS node context
if __name__ == "__main__":
    import sys
    import signal

    rospy.init_node('system_monitor_gui', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    app = QApplication(sys.argv)
    gui = SystemMonitorGUI()
    gui.show()

    sys.exit(app.exec_())