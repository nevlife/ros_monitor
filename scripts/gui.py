#!/usr/bin/env python3
import sys, json, os
from PySide6.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PySide6.QtCore import QTimer

class RosMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Latency Monitor")
        self.resize(600, 400)

        self.label = QLabel("Waiting for data...")
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000)

    def update_data(self):
        try:
            with open('/tmp/ros_latency_trace.json') as f:
                data = json.load(f)
            self.label.setText(str(data))
        except:
            self.label.setText("No data yet")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RosMonitorGUI()
    window.show()
    sys.exit(app.exec())
