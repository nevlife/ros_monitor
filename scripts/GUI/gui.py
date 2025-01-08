#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import signal
import rospy
from std_msgs.msg import Bool, Float32, Int8MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from PySide6.QtWidgets import QApplication, QLabel, QWidget, QGridLayout, QFrame, QHBoxLayout, QPushButton, QVBoxLayout
from PySide6.QtCore import Qt, QTimer


class SystemMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(1920, 1080)
        self.setWindowTitle("System Status Monitor")

        '''sys gride'''
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

        '''def gride'''
        self.def1 = None
    
        '''sensor gride'''
        self.status_list_sub = Subscriber('/sensor_status/sensor_status_bool', Int8MultiArray)
        
        self.lidar_val = 'N/A'
        self.gps_val = 'N/A'

        self.lider_status = QFrame()
        self.gps_status = QFrame()
        self.camera_status = QFrame()
        
        self.lider_label = QLabel(self.lidar_val)
        self.gps_label = QLabel(self.gps_val)
        self.camera_label = QLabel('Camera')
        
        '''button gride'''
        self.button_1 = QPushButton('1')
        self.button_2 = QPushButton('2')
        self.button_3 = QPushButton('3')
        
        self.button_1.clicked.connect(self.start_clicked)
        self.button_2.clicked.connect(self.stop_clicked)
        self.button_3.clicked.connect(self.reset_clicked)
        
        
        self.init_layout()
        self.init_msg_sync()
        self.init_style()
        #update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)
            
    def init_layout(self):
        '''
        QVBoxLayout (main_layout)(수직 배치)
        │
        ├── QFrame (button_container)
        │       └── QHBoxLayout (button_layout) (수평 배치)
        │           ├── QPushButton (button_1)
        │           ├── QPushButton (button_2)
        │           └── QPushButton (button_3)
        │
        └── QFrame (grid_container)
                └── QGridLayout (grid_layout)
                    │
                    ├── QFrame (sys_container)
                    │       └── QGridLayout (sys_layout)
                    │           ├── QLabel (cpu_usage_label)
                    │           ├── QLabel (cpu_freq_label)
                    │           ├── QLabel (gpu_usage_label)
                    │           └── QLabel (memory_percent_label)
                    │
                    ├── QFrame (def_container)
                    │       └── QGridLayout (def_layout)
                    │           └── QLabel ("Default Placeholder")
                    │
                    └── QFrame (sensor_container)
                            └── QGridLayout (sensor_layout)
                                ├── QFrame (lider_status)
                                ├── QFrame (gps_status)
                                ├── QFrame (camera_status)
                                ├── QLabel (lider_label)
                                ├── QLabel (gps_label)
                                └── QLabel (camera_label)

        '''
        main_layout = QVBoxLayout(self)  # 메인 레이아웃 설정

        
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
        sys_layout.addWidget(self.cpu_usage_label, 0, 0)
        sys_layout.addWidget(self.cpu_freq_label, 1, 0)
        sys_layout.addWidget(self.gpu_usage_label, 2, 0)
        sys_layout.addWidget(self.memory_percent_label, 3, 0)

        sys_container = QFrame()
        sys_container.setLayout(sys_layout)
        sys_container.setFrameShape(QFrame.Box)
        sys_container.setLineWidth(2)

        '''def grid'''
        def_layout = QGridLayout()
        def_layout.addWidget(QLabel("Default Placeholder"), 0, 0)

        def_container = QFrame()
        def_container.setLayout(def_layout)
        def_container.setFrameShape(QFrame.Box)
        def_container.setLineWidth(2)

        '''sensor grid'''
        sensor_layout = QGridLayout()
        sensor_layout.addWidget(self.lider_status, 0, 0)
        sensor_layout.addWidget(self.gps_status, 1, 0)
        sensor_layout.addWidget(self.camera_status, 2, 0)

        sensor_layout.addWidget(self.lider_label, 0, 1)
        sensor_layout.addWidget(self.gps_label, 1, 1)
        sensor_layout.addWidget(self.camera_label, 2, 1)

        sensor_container = QFrame()
        sensor_container.setLayout(sensor_layout)
        sensor_container.setFrameShape(QFrame.Box)
        sensor_container.setLineWidth(2)

        main_grid_layout = QGridLayout()
        main_grid_layout.addWidget(sys_container, 0, 0, 1, 1)
        main_grid_layout.addWidget(def_container, 0, 1, 3, 1)
        main_grid_layout.addWidget(sensor_container, 0, 2, 2, 1)

        grid_container = QFrame()
        grid_container.setLayout(main_grid_layout)

        main_layout.addWidget(button_container)
        main_layout.addWidget(grid_container)


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
        
        # print(f"CPU: {self.cpu_usage_val}")
        # print(f"CPU Frequency: {self.cpu_freq_val}")
        # print(f"GPU: {self.gpu_usage_val}")
        # print(f"Memory: {self.memory_percent_val}")
        # print(f'Lidar: {self.lidar_val}')
        # print(f'GPS: {self.gps_val}')
    
    def start_clicked(self):
        print("Start button clicked")

    def stop_clicked(self):
        print("Stop button clicked")

    def reset_clicked(self):
        print("Reset button clicked")
        
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
        
        self.button_1.setFixedSize(100,40)
        self.button_2.setFixedSize(100,40)
        self.button_3.setFixedSize(100,40)

def signal_handler(sig, frame):
    QApplication.quit()
    
def main():
    rospy.init_node('sys_status_gui', anonymous=True)
    app = QApplication(sys.argv)
    window = SystemMonitorGUI()
    window.show()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.on_shutdown(app.quit)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
