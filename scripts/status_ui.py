#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame, QPushButton, QHBoxLayout, QTableWidget, QTableWidgetItem
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from std_msgs.msg import Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber

from get_topic_info import MetricsManager


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

        node_info_lst = ['node names', 'Hz', 'Bandwidth', 'CPU Usage', 'RAM Usage']
        self.node_info_table = QTableWidget()
        self.node_info_table.setColumnCount(len(node_info_lst))
        
        self.node_info_table.setHorizontalHeaderLabels(node_info_lst)
        self.node_info_table.setColumnWidth(0, 500)  # Node Names 열
        self.node_info_table.setColumnWidth(1, 100)  # Hz 열
        self.node_info_table.setColumnWidth(2, 120)  # Bandwidth 열
        self.node_info_table.setColumnWidth(3, 300)  # CPU Usage 열
        self.node_info_table.setColumnWidth(4, 120)  # RAM Usage 열
        #self.node_info_table.setEditTriggers(QTableWidget.NoEditTriggers)
        
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
        node_info_layout = QVBoxLayout()
        node_info_layout.addWidget(self.node_info_table)
        
        node_info_container = QFrame()
        node_info_container.setLayout(node_info_layout)
        node_info_container.setFrameShape(QFrame.Box)
        node_info_container.setLineWidth(2)

        '''main layout'''
        main_grid_layout = QGridLayout()
        main_grid_layout.addWidget(sys_container, 0, 0, 1, 1)
        main_grid_layout.addWidget(node_info_container, 0, 1, 5, 4)
        
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
        #print(metrics)
        
        self.node_info_table.setRowCount(len(metrics))
        
        font = QFont()
        font.setPointSize(14)
        
        for row, (node, data) in enumerate(metrics.items()):
            #add layout
            node_item = QTableWidgetItem(node)
            node_item.setFont(font)
            self.node_info_table.setItem(row, 0, node_item)
            
            hz_item = QTableWidgetItem(f'{data["hz"]}')
            hz_item.setFont(font)
            self.node_info_table.setItem(row, 1, hz_item)
            
            bw_item = QTableWidgetItem(f'{data["bw"]}')
            bw_item.setFont(font)
            self.node_info_table.setItem(row, 2, bw_item)
            

    def start_clicked(self):
        rospy.loginfo('Start clicked')

    def stop_clicked(self):
        rospy.loginfo('Stop clicked')

    def reset_clicked(self):
        rospy.loginfo('Reset clicked')
        rospy.loginfo('Refreshing data...')
        self.update_metrics_data()
        
    def init_msgs_sync(self):
        self.sys_status_sub = Subscriber('/get_sys_status', Float32MultiArray) #total sys info
        ats = ApproximateTimeSynchronizer([self.sys_status_sub], queue_size=20, slop=1, allow_headerless=True)
        ats.registerCallback(self.sys_status_callback)
