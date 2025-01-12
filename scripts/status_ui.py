#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame, QPushButton, QHBoxLayout
from PySide6.QtCore import Qt, QTimer
from std_msgs.msg import Float32, Int8MultiArray, Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber


class SystemMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_components()
        self.init_layout()
        self.init_msgs_sync()
        
        self.node_status_received = {key: False for key in self.sensors.keys()}

    def init_components(self):
        
        '''sys info components'''
        self.sys_status_data = {
            'CPU Usage' : 'N/A',
            'CPU Freq' : 'N/A',
            'GPU Usage' : 'N/A',
            'Memory' : 'N/A',
        }
    
        self.sys_status_labels = {}
        for key in self.sys_status_data:
            '''create QLabel to sys_status_data'''
            label = QLabel(f'{key}: {self.sys_status_data[key]}')
            font = label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
            self.sys_status_labels[key] = label
            
        '''node componets'''        
        self.lidar_status_frame = QFrame()
        self.gps_fix_frame = QFrame()
        self.gps_fix_velocity_frame = QFrame()
        self.imu_frame = QFrame()
        
        self.zed_depth_info_frame = QFrame()
        self.zed_left_info_frame = QFrame()
        self.cam_info_frame = QFrame()

        def init_sensor_ui(name):
            frame = QFrame()
            frame.setFrameShape(QFrame.Box)
            frame.setFixedSize(50, 50)
            frame.setLineWidth(2)
            
            label = QLabel(name)
            font = label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
            
            return frame, label
        
        self.sensors ={
            'Lidar': init_sensor_ui('Lidar'),
            'GPS Fix': init_sensor_ui('GPS Fix'),
            'GPS Velocity': init_sensor_ui('GPS Velocity'),
            'IMU': init_sensor_ui('IMU'),
            'ZED Camera': init_sensor_ui('ZED Camera'),
            'Camera': init_sensor_ui('Camera')
        }
        
        '''buttons'''
        self.button_1 = QPushButton("Start")
        self.button_2 = QPushButton("Stop")
        self.button_3 = QPushButton("Reset")
        self.button_1.clicked.connect(self.start_clicked)
        self.button_2.clicked.connect(self.stop_clicked)
        self.button_3.clicked.connect(self.reset_clicked)
    
    
    def init_layout(self):
        """set layout, container"""
        main_layout = QVBoxLayout(self)

        '''button layout'''
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_1)
        button_layout.addWidget(self.button_2)
        button_layout.addWidget(self.button_3)
        button_layout.setSpacing(10)
        
        button_container = QFrame()
        button_container.setFixedHeight(50)
        button_layout.setAlignment(Qt.AlignLeft)
        button_container.setLayout(button_layout)

        '''sys grid'''
        sys_layout = QGridLayout()
        
        for row, label in enumerate(self.sys_status_labels.values()):
            sys_layout.addWidget(label, row, 0)

        sys_container = QFrame()
        sys_container.setLayout(sys_layout)
        sys_container.setFrameShape(QFrame.Box)
        sys_container.setLineWidth(2)
        
        '''def grid'''
        def_layout = QGridLayout()
        def_layout.addWidget(QLabel('Nothing'), 0, 0)
        
        def_container = QFrame()
        def_container.setLayout(def_layout)
        def_container.setFrameShape(QFrame.Box)
        def_container.setLineWidth(2)
        
        '''sensor grid'''
        sensor_layout = QGridLayout()
        
        for row, (name, (frame, label)) in enumerate(self.sensors.items()):
            sensor_layout.addWidget(frame, row, 0)
            sensor_layout.addWidget(label, row, 1)

        sensor_container = QFrame()
        sensor_container.setLayout(sensor_layout)
        sensor_container.setFrameShape(QFrame.Box)
        sensor_container.setLineWidth(2)
        
        main_grid_layout = QGridLayout()
        #시작 행, 시작 열, 행 범위, 열 범위
        main_grid_layout.addWidget(sys_container, 0, 0, 1, 1)
        main_grid_layout.addWidget(def_container, 0, 1, 5, 4)
        main_grid_layout.addWidget(sensor_container, 1, 0, 2, 1)
        
        grid_container = QFrame()
        grid_container.setLayout(main_grid_layout)
        
        main_layout.addWidget(button_container)
        main_layout.addWidget(grid_container)

    def sys_status_callback(self, sys_status_sub, node_connected_status_sub):
        #update status values
        print(node_connected_status_sub)
        expeted_node_status_len = 8
        data = list(node_connected_status_sub.data)
        data = data[:expeted_node_status_len] + [0.0] * (expeted_node_status_len - len(data))
        
        self.sys_status_data['CPU Usage'] = f'{sys_status_sub.data[0]:.2f}%'
        self.sys_status_data['CPU Freq'] = f'{sys_status_sub.data[1]:.2f} GHz'
        self.sys_status_data['GPU Usage'] = f'{sys_status_sub.data[2]:.2f}%'
        self.sys_status_data['Memory'] = f'{sys_status_sub.data[3]:.2f}%'

                
        for key, val in self.sys_status_data.items():
            self.sys_status_labels[key].setText(f'{key}: {val}')
            
        node_status = [
            (self.sensors["GPS Fix"][0], data[2]),
            (self.sensors["GPS Velocity"][0], data[3]),
            (self.sensors["IMU"][0], data[4]),
            (self.sensors["ZED Camera"][0], data[5]),
            (self.sensors["Camera"][0], data[6]),
            (self.sensors["Lidar"][0], data[7])
        ]
        
        for frame, status in node_status:
            if status:
                color = 'green'
            else:
                color = 'red'
            frame.setStyleSheet(f'background-color: {color};')

        for key in self.sensors.keys():
            self.node_status_received[key] = True

    def update_ui(self):
        """ui update"""
        return

    def start_clicked(self):
        print("Start clicked")

    def stop_clicked(self):
        print("Stop clicked")

    def reset_clicked(self):
        print("Reset clicked")

    def init_msgs_sync(self):
        self.sys_status_sub = Subscriber('/get_sys_status', Float32MultiArray)
        self.node_connected_status_sub = Subscriber('/node_connecter', Float32MultiArray)
        
        ats = ApproximateTimeSynchronizer(
            [self.sys_status_sub, self.node_connected_status_sub],
            queue_size=20,
            slop=1,
            allow_headerless=True
        )
        ats.registerCallback(self.sys_status_callback)
        
if __name__ == '__main__':
    SystemMonitorGUI()