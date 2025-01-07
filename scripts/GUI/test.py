#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import rospy
from std_msgs.msg import Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PySide6.QtCore import QTimer


class SystemMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("System Status Monitor")
        self.setGeometry(200, 200, 400, 300)

        layout = QVBoxLayout()

        self.cpu_label = QLabel("CPU Usage: N/A")
        self.freq_label = QLabel("CPU Frequency: N/A")
        self.gpu_label = QLabel("GPU Usage: N/A")
        self.memory_label = QLabel("Memory Usage: N/A")
        self.battery_label = QLabel("Battery: N/A")

        layout.addWidget(self.cpu_label)
        layout.addWidget(self.freq_label)
        layout.addWidget(self.gpu_label)
        layout.addWidget(self.memory_label)
        layout.addWidget(self.battery_label)

        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

        self.cpu_usage = "N/A"
        self.cpu_freq = "N/A"
        self.gpu_usage = "N/A"
        self.memory_percent = "N/A"
        self.battery_percent = "N/A"

    def update_status(self, cpu_usage, cpu_freq, gpu_usage, memory, battery):
        self.cpu_usage = f"{cpu_usage}%"
        self.cpu_freq = f"{cpu_freq} GHz"
        self.gpu_usage = f"{gpu_usage}%"
        self.memory_percent = f"{memory}%"
        self.battery_percent = f"{battery}%"

    def update_ui(self):
        self.cpu_label.setText(f"CPU Usage: {self.cpu_usage}")
        self.freq_label.setText(f"CPU Frequency: {self.cpu_freq}")
        self.gpu_label.setText(f"GPU Usage: {self.gpu_usage}")
        self.memory_label.setText(f"Memory Usage: {self.memory_percent}")
        self.battery_label.setText(f"Battery: {self.battery_percent}")


def sys_status_callback(cpu_usage, cpu_freq, gpu_usage, memory, battery):
    """ROS 메시지 콜백 함수"""
    window.update_status(cpu_usage.data, cpu_freq.data, gpu_usage.data, memory.data, battery.data)


if __name__ == "__main__":
    try:
        rospy.init_node('sys_status_gui', anonymous=True)

        app = QApplication(sys.argv)
        window = SystemMonitorGUI()

        cpu_usage = Subscriber('/sys_status/cpu_usage', Float32)
        cpu_freq = Subscriber('/sys_status/cpu_freq', Float32)
        gpu_usage = Subscriber('/sys_status/gpu_usage', Float32)
        memory = Subscriber('/sys_status/memory', Float32)
        battery = Subscriber('/sys_status/battery', Float32)

        ats = ApproximateTimeSynchronizer(
            [cpu_usage, cpu_freq, gpu_usage, memory, battery],
            queue_size=10,
            slop=1,
            allow_headerless=True
        )
        ats.registerCallback(sys_status_callback)

        def ros_spin():
            rospy.rostime.wallsleep(0.01)

        ros_timer = QTimer()
        ros_timer.timeout.connect(ros_spin)
        ros_timer.start(10)

        # GUI 실행
        window.show()
        sys.exit(app.exec())

    except rospy.ROSInterruptException:
        pass
