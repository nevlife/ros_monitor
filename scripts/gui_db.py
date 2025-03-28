#!/usr/bin/env python3
import sys
import os
import sqlite3
import signal
import psutil
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel, QTableWidget,
    QTableWidgetItem, QHeaderView
)
from PySide6.QtCore import QTimer, Qt


class DBReader:
    def __init__(self, db_path):
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row

    def fetch_latest_total_resource(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM total_resource ORDER BY timestamp DESC LIMIT 1")
        return cur.fetchone()

    def fetch_latest_node_resource(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM node_resource ORDER BY timestamp DESC")
        rows = cur.fetchall()
        nodes = {}
        for row in rows:
            if row['node_name'] not in nodes:
                nodes[row['node_name']] = {"cpu": row['cpu'], "mem": row['mem']}
        return nodes

    def fetch_latest_topic_hzbw(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM topic_hzbw ORDER BY timestamp DESC")
        rows = cur.fetchall()
        topics = {}
        for row in rows:
            if row['topic_name'] not in topics:
                topics[row['topic_name']] = {"hz": row['hz'], "bw": row['bw']}
        return topics

    def fetch_latest_gpu_pmon(self):
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM gpu_pmon ORDER BY timestamp DESC")
        rows = cur.fetchall()
        processes = {}
        for row in rows:
            pid = row['pid']
            if pid not in processes:
                processes[pid] = {
                    "type": row['type'],
                    "sm": row['sm_usage'],
                    "mem": row['mem_usage'],
                    "cmd": row['command']
                }
        return processes


class RosMonitor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Monitor (DB)")
        self.resize(1200, 1080)

        self.db = DBReader(os.path.expanduser("~/catkin_ws/src/ros_monitor/data/data.db"))
        self.init_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(1000)

    def init_ui(self):
        self.layout = QGridLayout(self)

        self.cpu_label = QLabel("CPU Info")
        self.mem_label = QLabel("Memory Info")
        self.gpu_label = QLabel("GPU Info")

        label_layout = QVBoxLayout()
        label_layout.addWidget(self.cpu_label)
        label_layout.addWidget(self.mem_label)
        label_layout.addWidget(self.gpu_label)
        self.layout.addLayout(label_layout, 0, 0, 1, 1)

        self.topics_table = QTableWidget(0, 3)
        self.topics_table.setHorizontalHeaderLabels(['Topic', 'Hz', 'Bandwidth'])
        self.topics_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.layout.addWidget(self.topics_table, 3, 0, 3, 1)

        self.nodes_table = QTableWidget(0, 3)
        self.nodes_table.setHorizontalHeaderLabels(['Node', 'CPU (%)', 'Memory'])
        self.nodes_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.layout.addWidget(self.nodes_table, 3, 1, 3, 1)

        self.gpu_table = QTableWidget(0, 5)
        self.gpu_table.setHorizontalHeaderLabels(['PID', 'Proc Name', 'Type', 'SM Usage', 'Command'])
        self.gpu_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.gpu_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.layout.addWidget(self.gpu_table, 7, 0, 2, 2)

    def refresh(self):
        self.update_total_resource()
        self.update_topic_table()
        self.update_node_table()
        self.update_gpu_table()

    def update_total_resource(self):
        row = self.db.fetch_latest_total_resource()
        if not row:
            return
        self.cpu_label.setText(f"CPU: {row['cpu_usage_percent']:.2f}% | Temp: {row['cpu_temp']:.1f}°C")
        self.mem_label.setText(f"Memory: {row['mem_used']:.1f}/{row['mem_total']:.1f} MB ({row['mem_usage_percent']:.1f}%)")
        self.gpu_label.setText(f"GPU: {row['gpu_usage_percent']:.1f}% | Temp: {row['gpu_temp']:.1f}°C | Mem Usage: {row['gpu_mem_usage']:.1f}%")

    def update_topic_table(self):
        data = self.db.fetch_latest_topic_hzbw()
        self.topics_table.setRowCount(len(data))
        for row, (name, info) in enumerate(data.items()):
            self._set_table_row(self.topics_table, row, [name, f"{info['hz']:.2f}", info['bw']])

    def update_node_table(self):
        data = self.db.fetch_latest_node_resource()
        self.nodes_table.setRowCount(len(data))
        for row, (name, info) in enumerate(data.items()):
            self._set_table_row(self.nodes_table, row, [name, f"{info['cpu']:.2f}", info['mem']])

    def update_gpu_table(self):
        data = self.db.fetch_latest_gpu_pmon()
        self.gpu_table.setRowCount(len(data))
        for row, (pid, info) in enumerate(data.items()):
            try:
                proc_name = psutil.Process(int(pid)).name()
            except:
                proc_name = "-"
            self._set_table_row(self.gpu_table, row, [
                pid, proc_name, info['type'], f"{info['sm']:.1f}", info['cmd']
            ])

    def _set_table_row(self, table, row, values):
        for col, val in enumerate(values):
            item = QTableWidgetItem(val)
            item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            table.setItem(row, col, item)


def signal_handler(sig, frame):
    QApplication.quit()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    app = QApplication(sys.argv)
    window = RosMonitor()
    window.show()
    sys.exit(app.exec())
