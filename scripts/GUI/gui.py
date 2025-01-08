#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import rospy
from std_msgs.msg import Bool, Float32, Int8MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from PySide6.QtWidgets import QApplication, QLabel, QWidget, QGridLayout, QFrame, QVBoxLayout
from PySide6.QtCore import QTimer


class SystemMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(1920, 1080)
        self.setWindowTitle("System Status Monitor")

        '''sys_gride'''
        self.cpu_usage_sub = Subscriber('/sys_status/cpu_usage', Float32)
        self.cpu_freq_sub = Subscriber('/sys_status/cpu_freq', Float32)
        self.gpu_usage_sub = Subscriber('/sys_status/gpu_usage', Float32)
        self.memory_sub = Subscriber('/sys_status/memory', Float32)

        #status value
        self.cpu_usage_val = 'N/A'
        self.cpu_freq_val = 'N/A'
        self.gpu_usage_val = 'N/A'
        self.memory_percent_val = 'N/A'

        #output labels
        self.cpu_usage_label = QLabel(self.cpu_usage_val)
        self.cpu_freq_label = QLabel(self.cpu_freq_val)
        self.gpu_usage_label = QLabel(self.gpu_usage_val)
        self.memory_percent_label = QLabel(self.memory_percent_val)

        '''def_gride'''
        self.def1 = None
    
        '''sensor_gride'''
        self.status_list_sub = Subscriber('/sensor_status/sensor_status_bool', Int8MultiArray)
        
        self.lidar_val = 'N/A'
        self.gps_val = 'N/A'

        
        self.lider_status = QFrame()
        self.gps_status = QFrame()
        self.camera_status = QFrame()
        
        self.lider_label = QLabel(self.lidar_val)
        self.gps_label = QLabel(self.gps_val)
        self.camera_label = QLabel('Camera')
        
        self.init_layout()
        self.init_msg_sync()
        self.init_style()
        #update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def init_layout(self):
        main_grid_layout = QGridLayout()

        '''sys_grid'''
        sys_grid = QGridLayout()
        sys_grid.addWidget(self.cpu_usage_label, 0, 0)
        sys_grid.addWidget(self.cpu_freq_label, 1, 0)
        sys_grid.addWidget(self.gpu_usage_label, 2, 0)
        sys_grid.addWidget(self.memory_percent_label, 3, 0)

        sys_container = QFrame()
        sys_container.setLayout(sys_grid)
        sys_container.setFrameShape(QFrame.Box)
        sys_container.setFrameShadow(QFrame.Raised)
        sys_container.setLineWidth(2)

        '''def_grid'''
        def_grid = QGridLayout()
        def_grid.addWidget(QLabel("Default Placeholder"), 0, 0)
        def_container = QFrame()
        def_container.setLayout(def_grid)
        def_container.setFrameShape(QFrame.Box)
        def_container.setFrameShadow(QFrame.Raised)
        def_container.setLineWidth(2)

        '''sensor_grid'''
        sensor_grid = QGridLayout()
        sensor_grid.addWidget(self.lider_status, 0, 0)
        sensor_grid.addWidget(self.gps_status, 1, 0)
        sensor_grid.addWidget(self.camera_status, 2, 0)
        
        sensor_grid.addWidget(self.lider_label, 0, 1)
        sensor_grid.addWidget(self.gps_label, 1, 1)
        sensor_grid.addWidget(self.camera_label, 2, 1)

        
        sensor_container = QFrame()
        sensor_container.setLayout(sensor_grid)
        sensor_container.setFrameShape(QFrame.Box)
        sensor_container.setFrameShadow(QFrame.Raised)
        sensor_container.setLineWidth(2)

        #add container
        main_grid_layout.addWidget(sys_container, 0, 0, 1, 1)
        main_grid_layout.addWidget(def_container, 0, 1, 3, 1)
        main_grid_layout.addWidget(sensor_container, 0, 2, 2, 1)

        self.setLayout(main_grid_layout)

    def sys_status_callback(self, cpu_usage, cpu_freq, gpu_usage, memory, status_list):
        #update status values
        self.cpu_usage_val = f"{cpu_usage.data:.2f}%"
        self.cpu_freq_val = f"{cpu_freq.data:.2f} GHz"
        self.gpu_usage_val = f"{gpu_usage.data:.2f}%"
        self.memory_percent_val = f"{memory.data:.2f}%"

        if bool(status_list.data[0]):
            lider_c = 'background-color: green;'
        else:
            lider_c = 'background-color: red;'
        
        if bool(status_list.data[1]):
            gps_c = 'background-color: green;'
        else:
            gps_c = 'background-color: red;'
            
        if bool(status_list.data[2]):
            camera_c = 'background-color: green;'
        else:
            camera_c = 'background-color: red;'
            
        self.lider_status.setStyleSheet(lider_c)
        self.gps_status.setStyleSheet(gps_c)
        self.camera_status.setStyleSheet(camera_c)

    def update_ui(self):
        #update output label
        '''sys_layout'''
        self.cpu_usage_label.setText(f"CPU: {self.cpu_usage_val}")
        self.cpu_freq_label.setText(f"CPU Frequency: {self.cpu_freq_val}")
        self.gpu_usage_label.setText(f"GPU: {self.gpu_usage_val}")
        self.memory_percent_label.setText(f"Memory: {self.memory_percent_val}")

        '''sensor layout'''
        self.lider_label.setText(f'Lidar: {self.lidar_val}')
        self.gps_label.setText(f'GPS: {self.gps_val}')
        
    def init_msg_sync(self):
        ats = ApproximateTimeSynchronizer(
            [self.cpu_usage_sub, self.cpu_freq_sub, self.gpu_usage_sub, self.memory_sub, self.status_list_sub],
            queue_size=20,
            slop=1,
            allow_headerless=True
        )
        ats.registerCallback(self.sys_status_callback)

    def init_style(self):
        labels = [self.cpu_usage_label, self.cpu_freq_label, self.gpu_usage_label, self.memory_percent_label]
        
        for label in labels:
            font =label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
        
        labels = [self.lider_label, self.gps_label, self.camera_label]
        
        for label in labels:
            font =label.font()
            font.setPointSize(15)
            font.setBold(True)
            label.setFont(font)
            
        sensor_box = [self.lider_status, self.gps_status, self.camera_status]
        
        for box in sensor_box:
            box.setFrameShape(QFrame.Box)
            box.setFixedSize(50, 50)
            box.setLineWidth(2)
        

def main():
    rospy.init_node('sys_status_gui', anonymous=True)
    app = QApplication(sys.argv)
    window = SystemMonitorGUI()
    window.show()
    rospy.on_shutdown(app.quit)  # 종료 처리
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
