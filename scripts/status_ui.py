#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame, QPushButton, QHBoxLayout
from PySide6.QtCore import Qt, QTimer
from std_msgs.msg import Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber

from dynamic_node_connect import MetricsManager


class SystemMonitorGUI(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        #rospy.init_node('system_monitor_gui', anonymous=True)

        self.metrics_manager = MetricsManager()
        self.metrics_labels = {}

        self.init_components()
        self.init_layout()
        self.init_msgs_sync()

        #update metrics data
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_metrics_data)
        self.update_timer.start(1000)  # 1초마다 실행

    def init_components(self):

        # sys info
        self.sys_status_data = {
            'CPU Usage': 'N/A',
            'CPU Freq': 'N/A',
            'GPU Usage': 'N/A',
            'Memory': 'N/A',
        }
        self.sys_status_labels = {}
        for key in self.sys_status_data:
            label = QLabel(f'{key}: {self.sys_status_data[key]}')
            font = label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
            self.sys_status_labels[key] = label

        self.button_1 = QPushButton('Start')
        self.button_2 = QPushButton('Stop')
        self.button_3 = QPushButton('Reset')
        self.button_1.clicked.connect(self.start_clicked)
        self.button_2.clicked.connect(self.stop_clicked)
        self.button_3.clicked.connect(self.reset_clicked)

    def init_layout(self):
        '''set layout'''
        main_layout = QVBoxLayout(self)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_1)
        button_layout.addWidget(self.button_2)
        button_layout.addWidget(self.button_3)
        button_layout.setSpacing(10)
        
        button_container = QFrame()
        button_container.setFixedHeight(50)
        button_layout.setAlignment(Qt.AlignLeft)
        button_container.setLayout(button_layout)

        '''sys info layout'''
        sys_layout = QGridLayout()
        for row, label in enumerate(self.sys_status_labels.values()):
            sys_layout.addWidget(label, row, 0)
            
        sys_container = QFrame()
        sys_container.setLayout(sys_layout)
        sys_container.setFrameShape(QFrame.Box)
        sys_container.setLineWidth(2)

        '''node info layout'''
        self.node_info_layout = QGridLayout()
        self.node_info_container = QFrame()
        self.node_info_container.setLayout(self.node_info_layout)
        self.node_info_container.setFrameShape(QFrame.Box)
        self.node_info_container.setLineWidth(2)

        '''main layout'''
        main_grid_layout = QGridLayout()
        main_grid_layout.addWidget(sys_container, 0, 0, 1, 1)
        main_grid_layout.addWidget(self.node_info_container, 0, 1, 5, 4)
        
        grid_container = QFrame()
        grid_container.setLayout(main_grid_layout)
        
        main_layout.addWidget(button_container)
        main_layout.addWidget(grid_container)

    def sys_status_callback(self, sys_status_sub):
        '''update sys info'''
        self.sys_status_data['CPU Usage'] = f'{sys_status_sub.data[0]:.2f}%'
        self.sys_status_data['CPU Freq'] = f'{sys_status_sub.data[1]:.2f} GHz'
        self.sys_status_data['GPU Usage'] = f'{sys_status_sub.data[2]:.2f}%'
        self.sys_status_data['Memory'] = f'{sys_status_sub.data[3]:.2f}%'
        for key, val in self.sys_status_data.items():
            self.sys_status_labels[key].setText(f'{key}: {val}')

    def update_metrics_data(self):
        '''update node info'''
        metrics = self.metrics_manager.get_metrics()
        for row, (node, data) in enumerate(metrics.items()):
            #init label
            if node not in self.metrics_labels:
                hz_label = QLabel(f'{node} Hz: {data["hz"]}hz')
                bandwidth_label = QLabel(f'{node} Bandwidth: {data["bandwidth"]}')
                print(hz_label)
                
                #add layout
                self.node_info_layout.addWidget(hz_label, row, 0)
                self.node_info_layout.addWidget(bandwidth_label, row, 1)

                #save label
                self.metrics_labels[node] = {
                    'hz_label': hz_label,
                    'bandwidth_label': bandwidth_label,
                }

            #label update
            self.metrics_labels[node]['hz_label'].setText(f'{node} Hz: {data["hz"]}')
            self.metrics_labels[node]['bandwidth_label'].setText(f'{node} Bandwidth: {data["bandwidth"]}')

    def start_clicked(self):
        rospy.loginfo('Start clicked')

    def stop_clicked(self):
        rospy.loginfo('Stop clicked')

    def reset_clicked(self):
        rospy.loginfo('Reset clicked')
        rospy.loginfo('Refreshing data...')
        self.update_metrics_data()
        
    def init_msgs_sync(self):
        self.sys_status_sub = Subscriber('/get_sys_status', Float32MultiArray)
        ats = ApproximateTimeSynchronizer([self.sys_status_sub], queue_size=20, slop=1, allow_headerless=True)
        ats.registerCallback(self.sys_status_callback)
