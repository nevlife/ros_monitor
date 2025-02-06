#!/usr/bin/env python3
import sys
import os
import json
import signal
import rospy
from PySide6.QtWidgets import (
    QWidget, 
    QVBoxLayout, 
    QGridLayout,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QApplication
)
from PySide6.QtCore import QTimer


def signal_handler(sig, frame):
    QApplication.quit()
    sys.exit(0)


class RosMonitor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS Monitor UI')
        self.resize(1200, 800)

        # JSON 데이터 저장
        self.total_resource_data = {}
        self.topic_hzbw_data = {}
        self.node_resource_usage_data = {}

        # JSON 파일 경로 설정
        self.file_path = os.path.expanduser("~/catkin_ws/src/vehicle_diag/data/diag.json")

        # UI 초기화
        self.init_ui()

        # 타이머 설정 (1초마다 JSON 파일 읽고 UI 갱신)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui_from_json)
        self.timer.start(1000)  # 1초마다 실행

    def init_ui(self):
        """ UI 초기화 """
        self.layout = QVBoxLayout(self)

        # CPU 정보
        self.cpu_label = QLabel("CPU Info: Loading...")
        self.layout.addWidget(self.cpu_label)

        # Memory 정보
        self.mem_label = QLabel("Memory Info: Loading...")
        self.layout.addWidget(self.mem_label)

        # GPU 정보
        self.gpu_label = QLabel("GPU Info: Loading...")
        self.layout.addWidget(self.gpu_label)

        # Topics Hz/BW 테이블
        self.topics_table = QTableWidget(0, 3)
        self.topics_table.setHorizontalHeaderLabels(["Topic", "Hz", "BW (KB/s)"])
        self.topics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.layout.addWidget(self.topics_table)

        # Node 리소스 테이블
        self.nodes_table = QTableWidget(0, 3)
        self.nodes_table.setHorizontalHeaderLabels(["Node", "CPU (%)", "Memory (MB)"])
        self.nodes_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.layout.addWidget(self.nodes_table)

    def update_ui_from_json(self):
        """ JSON 파일을 읽고 UI를 업데이트 """
        if not os.path.exists(self.file_path):
            return

        try:
            with open(self.file_path, "r") as file:
                data = json.load(file)
                self.total_resource_data = data.get("total_resource", {})
                self.topic_hzbw_data = data.get("topics_hzbw", {})
                self.node_resource_usage_data = data.get("node_resource_usage", {})

                self.update_total_resource_ui()
                self.update_topics_table()
                self.update_nodes_table()
        except Exception as e:
            print(f"Error reading JSON file: {e}")

    def update_total_resource_ui(self):
        """ CPU, Memory, GPU 정보 UI 업데이트 """
        self.cpu_label.setText(
            f"CPU: {self.total_resource_data.get('cpu_usage_percent', 'N/A')}% | "
            f"Temp: {self.total_resource_data.get('cpu_temp', 'N/A')}°C | "
            f"Load Avg: {self.total_resource_data.get('cpu_load_1min', 'N/A')}, "
            f"{self.total_resource_data.get('cpu_load_5min', 'N/A')}, "
            f"{self.total_resource_data.get('cpu_load_15min', 'N/A')}"
        )

        self.mem_label.setText(
            f"Memory: {self.total_resource_data.get('mem_used', 'N/A')}MB / "
            f"{self.total_resource_data.get('mem_total', 'N/A')}MB "
            f"({self.total_resource_data.get('mem_usage_percent', 'N/A')}%)"
        )

        self.gpu_label.setText(
            f"GPU: {self.total_resource_data.get('gpu_usage_percent', 'N/A')}% | "
            f"Memory: {self.total_resource_data.get('gpu_mem_used', 'N/A')}MB / "
            f"{self.total_resource_data.get('gpu_mem_total', 'N/A')}MB "
            f"({self.total_resource_data.get('gpu_mem_usage', 'N/A')}%) | "
            f"Temp: {self.total_resource_data.get('gpu_temp', 'N/A')}°C"
        )

    def update_topics_table(self):
        """ Topics Hz/BW 테이블 업데이트 """
        self.topics_table.setRowCount(len(self.topic_hzbw_data))
        for row, (topic, metrics) in enumerate(self.topic_hzbw_data.items()):
            self.topics_table.setItem(row, 0, QTableWidgetItem(topic))
            self.topics_table.setItem(row, 1, QTableWidgetItem(str(round(metrics.get("hz", 0), 2))))
            self.topics_table.setItem(row, 2, QTableWidgetItem(str(round(metrics.get("bw", 0), 2))))

    def update_nodes_table(self):
        """ Node 리소스 테이블 업데이트 """
        self.nodes_table.setRowCount(len(self.node_resource_usage_data))
        for row, (node, resources) in enumerate(self.node_resource_usage_data.items()):
            self.nodes_table.setItem(row, 0, QTableWidgetItem(node))
            self.nodes_table.setItem(row, 1, QTableWidgetItem(str(resources.get("cpu", "N/A"))))
            self.nodes_table.setItem(row, 2, QTableWidgetItem(str(round(resources.get("mem", 0) / (1024 * 1024), 2))))  # MB 단위 변환


# Ensure this is run in a ROS node context
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    app = QApplication(sys.argv)
    window = RosMonitor()
    window.show()

    sys.exit(app.exec())
