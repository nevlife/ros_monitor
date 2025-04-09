import json
import atexit
from collections import deque

import psutil
import rospy
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QApplication, QPlainTextEdit
)
from PySide6.QtGui import QFont, QColor
from PySide6.QtCore import Qt, QTimer
from std_msgs.msg import Float32MultiArray, String

import ros_tracker  # ros_tracker 모듈은 find_topic_connections, find_node_connections, find_node_pid 등의 함수를 제공한다고 가정

MAX_LOG_LINES = 500

class RosMonitor(QWidget):
    def __init__(self):
        super().__init__()
        # ROS 노드 초기화
        rospy.init_node('ros_monitor_ui', anonymous=True)
        self.setWindowTitle('ROS Monitor UI')
        self.resize(1920, 1080)

        # 내부 상태 저장용 딕셔너리
        self.total_resource_data = {}
        self.topic_hzbw_data = {}
        self.nodes_resource_data = {}
        self.gpu_proc_data = {}

        # CPU, Memory, GPU 라벨
        self.cpu_label = QLabel('CPU Info: -')
        self.mem_label = QLabel('Memory Info: -')
        self.gpu_label = QLabel('GPU Info: -')

        self.log_lines = deque(maxlen=MAX_LOG_LINES)
        self.log_widget = QPlainTextEdit()
        self.log_widget.setReadOnly(True)

        self.bold_font = QFont()
        self.bold_font.setBold(True)

        self.init_ui()
        self.setup_subscribers()

        # 1초마다 UI 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(1000)

    def init_ui(self):
        logical_cores = psutil.cpu_count(logical=True)
        self.layout = QGridLayout(self)

        # 로그 위젯: 왼쪽에 배치
        self.layout.addWidget(self.log_widget, 0, 0, 10, 1)

        # 상단 라벨 레이아웃
        label_layout = QVBoxLayout()
        label_layout.addWidget(self.cpu_label)
        label_layout.addWidget(self.mem_label)
        label_layout.addWidget(self.gpu_label)
        self.layout.addLayout(label_layout, 0, 1, 1, 2)

        # 토픽 테이블
        self.topics_table = QTableWidget(0, 3)
        self.topics_table.setHorizontalHeaderLabels(['Topic name', 'Hz', 'Bandwidth'])
        self.topics_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.topics_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.topics_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.layout.addWidget(self.topics_table, 1, 1, 3, 1)
        self.topics_table.cellClicked.connect(self.topic_clicked)

        # 노드 테이블
        self.nodes_table = QTableWidget(0, 3)
        self.nodes_table.setHorizontalHeaderLabels([
            'Node name',
            f'CPU (%)\nLogical Core: {logical_cores}',
            'Ram'
        ])
        self.nodes_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.nodes_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.nodes_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.layout.addWidget(self.nodes_table, 1, 2, 3, 1)
        self.nodes_table.cellClicked.connect(self.node_clicked)

        # GPU 프로세스 테이블
        self.gpu_proc_table = QTableWidget(0, 9)
        self.gpu_proc_table.setHorizontalHeaderLabels([
            'pid', 'type', 'sm', 'mem', 'enc', 'dec', 'jpg', 'ofa', 'command'
        ])
        for i in range(8):
            self.gpu_proc_table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeToContents)
        self.gpu_proc_table.horizontalHeader().setSectionResizeMode(8, QHeaderView.Stretch)
        self.layout.addWidget(self.gpu_proc_table, 4, 1, 3, 2)

        self.setLayout(self.layout)

    def setup_subscribers(self):
        rospy.Subscriber('/total_resource', Float32MultiArray, self.total_resource_callback)
        rospy.Subscriber('/topics_hzbw', String, self.topic_hzbw_callback)
        rospy.Subscriber('/nodes_resource', String, self.nodes_resource_callback)
        rospy.Subscriber('/gpu_pmon', String, self.gpu_pmon_callback)

    def update_ui(self):
        self.update_labels()
        self.update_topics_table()
        self.update_nodes_table()
        self.update_gpu_table()

    def total_resource_callback(self, msg):
        fields = [
            'cpu_user', 'cpu_nice', 'cpu_system', 'cpu_idle', 'cpu_iowait',
            'cpu_irq', 'cpu_softirq', 'cpu_steal', 'cpu_guest', 'cpu_guest_nice',
            'cpu_usage_percent', 'cpu_temp',
            'cpu_load_1min', 'cpu_load_5min', 'cpu_load_15min',
            'mem_used', 'mem_total', 'mem_usage_percent',
            'gpu_usage_percent', 'gpu_mem_used', 'gpu_mem_total', 'gpu_mem_usage', 'gpu_temp'
        ]
        self.total_resource_data = {name: value for name, value in zip(fields, msg.data)}

    def topic_hzbw_callback(self, msg):
        data = json.loads(msg.data)
        temp = {}
        for item in data:
            temp[item['topic']] = {
                'hz': item['hz'],
                'bw': item['bw'],
                'target_hz': item.get('target_hz', None)
            }
        self.topic_hzbw_data = temp

    def nodes_resource_callback(self, msg):
        data = json.loads(msg.data)
        temp = {}
        for item in data:
            temp[item['node']] = {
                'cpu': item['cpu'],
                'mem': item['mem']
            }
        self.nodes_resource_data = temp

    def gpu_pmon_callback(self, msg):
        data = json.loads(msg.data)
        temp = {}
        for item in data:
            temp[item['pid']] = {
                'gpu_idx': item['gpu_idx'],
                'type': item['type'],
                'sm': item['sm'],
                'mem': item['mem'],
                'enc': item['enc'],
                'dec': item['dec'],
                'jpg': item['jpg'],
                'ofa': item['ofa'],
                'command': item['command'],
            }
        self.gpu_proc_data = temp

    def update_labels(self):
        g = self.total_resource_data.get
        cpu_usage = g('cpu_usage_percent', -1)
        cpu_temp = g('cpu_temp', -1)
        load1 = g('cpu_load_1min', -1)
        load5 = g('cpu_load_5min', -1)
        load15 = g('cpu_load_15min', -1)

        mem_used = g('mem_used', -1)
        mem_total = g('mem_total', -1)
        mem_percent = g('mem_usage_percent', -1)

        gpu_usage = g('gpu_usage_percent', -1)
        gpu_mem_used = g('gpu_mem_used', -1)
        gpu_mem_total = g('gpu_mem_total', -1)
        gpu_temp = g('gpu_temp', -1)

        self.cpu_label.setText(
            f'CPU: {cpu_usage:.2f}% | Temp: {cpu_temp:.1f}C | Load: {load1:.2f}, {load5:.2f}, {load15:.2f}'
        )
        self.mem_label.setText(
            f'Mem: {mem_used:.1f}/{mem_total:.1f} MB ({mem_percent:.2f}%)'
        )
        self.gpu_label.setText(
            f'GPU: {gpu_usage:.2f}% | Mem: {gpu_mem_used:.1f}/{gpu_mem_total:.1f} MB | Temp: {gpu_temp:.1f}C'
        )

    def update_topics_table(self):
        self.topics_table.setRowCount(len(self.topic_hzbw_data))
        for row, (topic, values) in enumerate(self.topic_hzbw_data.items()):
            hz_value = values.get('hz', -1)
            bw_value = values.get('bw', '0 B/s')
            target_hz = values.get('target_hz', None)
            color = None

            if target_hz is not None:
                try:
                    target_hz = float(target_hz)
                    ratio = hz_value / target_hz if target_hz != 0 else 9999
                    if ratio < 0.6:
                        rospy.logerr(f'[Hz Error] {topic}: {hz_value} Hz / 60% of target hz ({target_hz})')
                        self.append_log(f'[Hz Error] {topic}: {hz_value} Hz / 60% of target hz ({target_hz})')
                        color = QColor('red')
                    elif ratio < 0.8:
                        rospy.logwarn(f'[Hz Warn] {topic}: {hz_value} Hz / 80% of target hz ({target_hz})')
                        self.append_log(f'[Hz Warn] {topic}: {hz_value} Hz / 80% of target hz ({target_hz})')
                        color = QColor('orange')
                except ValueError:
                    pass

            topic_item = QTableWidgetItem(topic)
            hz_item = QTableWidgetItem(str(hz_value))
            bw_item = QTableWidgetItem(str(bw_value))
            for item in [topic_item, hz_item, bw_item]:
                item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                if color:
                    item.setForeground(color)
                    item.setFont(self.bold_font)

            self.topics_table.setItem(row, 0, topic_item)
            self.topics_table.setItem(row, 1, hz_item)
            self.topics_table.setItem(row, 2, bw_item)

    def update_nodes_table(self):
        self.nodes_table.setRowCount(len(self.nodes_resource_data))
        for row, (node, resources) in enumerate(self.nodes_resource_data.items()):
            cpu_value = resources.get('cpu', -1)
            mem_value = resources.get('mem', 'N/A')
            node_item = QTableWidgetItem(node)
            cpu_item = QTableWidgetItem(f'{cpu_value:.2f}')
            mem_item = QTableWidgetItem(str(mem_value))
            for item in [node_item, cpu_item, mem_item]:
                item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.nodes_table.setItem(row, 0, node_item)
            self.nodes_table.setItem(row, 1, cpu_item)
            self.nodes_table.setItem(row, 2, mem_item)

    def update_gpu_table(self):
        self.gpu_proc_table.setRowCount(len(self.gpu_proc_data))
        for row, (pid, info) in enumerate(self.gpu_proc_data.items()):
            pid_item   = QTableWidgetItem(pid)
            type_item  = QTableWidgetItem(info.get('type', '-'))
            sm_item    = QTableWidgetItem(str(info.get('sm', -1)))
            mem_item   = QTableWidgetItem(str(info.get('mem', -1)))
            enc_item   = QTableWidgetItem(str(info.get('enc', -1)))
            dec_item   = QTableWidgetItem(str(info.get('dec', -1)))
            jpg_item   = QTableWidgetItem(str(info.get('jpg', -1)))
            ofa_item   = QTableWidgetItem(str(info.get('ofa', -1)))
            cmd_item   = QTableWidgetItem(info.get('command', '-'))
            for item in [pid_item, type_item, sm_item, mem_item, enc_item, dec_item, jpg_item, ofa_item, cmd_item]:
                item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.gpu_proc_table.setItem(row, 0, pid_item)
            self.gpu_proc_table.setItem(row, 1, type_item)
            self.gpu_proc_table.setItem(row, 2, sm_item)
            self.gpu_proc_table.setItem(row, 3, mem_item)
            self.gpu_proc_table.setItem(row, 4, enc_item)
            self.gpu_proc_table.setItem(row, 5, dec_item)
            self.gpu_proc_table.setItem(row, 6, jpg_item)
            self.gpu_proc_table.setItem(row, 7, ofa_item)
            self.gpu_proc_table.setItem(row, 8, cmd_item)

    def topic_clicked(self, row, col):
        topic_item = self.topics_table.item(row, 0)
        if not topic_item:
            return
        clicked_topic = topic_item.text()
        pubs, subs = ros_tracker.find_topic_connections(clicked_topic)
        print(pubs, subs)
        self.append_log("\n\n===================================================")
        self.append_log(f"<b>Topic : \"{clicked_topic}\"</b>")
        self.append_log(f"Nodes publishing this topic: {pubs}")
        self.append_log(f"Nodes subscribing to this topic: {subs}")

        self.append_log("\n<b>Nodes publishing the clicked topic</b>\n")
        if pubs:
            for pub_node in pubs:
                pub_topics, sub_topics = ros_tracker.find_node_connections(pub_node)
                self.append_log(f"<b>Node : {pub_node}</b>")
                self.append_log("Publishes :")
                for topic in pub_topics:
                    self.append_log(f'      {topic}')
                self.append_log("Subscribes :")
                for topic in sub_topics:
                    self.append_log(f'      {topic}')
                self.append_log("\n")
        else:
            self.append_log("Not Found")

        self.append_log("\n<b>Nodes subscribing to the clicked topic</b>\n")
        if subs:
            for sub_node in subs:
                pub_topics, sub_topics = ros_tracker.find_node_connections(sub_node)
                self.append_log(f"<b>Node : {sub_node}</b>")
                self.append_log("Publishes :")
                for topic in pub_topics:
                    self.append_log(f'      {topic}')
                self.append_log("Subscribes :")
                for topic in sub_topics:
                    self.append_log(f'      {topic}')
                self.append_log("\n")
        else:
            self.append_log("Not Found")

    def node_clicked(self, row, col):
        node_item = self.nodes_table.item(row, 0)
        if not node_item:
            return
        node_name = node_item.text()
        self.append_log(f'\n\n===================================================')
        self.append_log(f'<b>Node Clicked:</b> "{node_name}"')
        pub_topics, sub_topics = ros_tracker.find_node_connections(node_name)
        self.append_log('<b>Publishes topic:</b>')
        for topic in pub_topics:
            self.append_log(f'   {topic}')
        self.append_log('<b>Subscribes topic:</b>')
        for topic in sub_topics:
            self.append_log(f'   {topic}')

        pid, cmdline = ros_tracker.find_node_pid(node_name)
        if pid:
            self.append_log(f'<b>PID:</b> {pid}')
            self.append_log(f'<b>Command:</b> {cmdline}')
        else:
            self.append_log('<b>PID:</b> Not Found')

    def append_log(self, text: str):
        scroll_bar = self.log_widget.verticalScrollBar()
        at_bottom = scroll_bar.value() == scroll_bar.maximum()
        self.log_lines.append(text)
        if len(self.log_lines) == MAX_LOG_LINES:
            self.log_widget.setPlainText('\n'.join(self.log_lines))
        else:
            self.log_widget.appendPlainText(text)
        if at_bottom:
            self.log_widget.verticalScrollBar().setValue(self.log_widget.verticalScrollBar().maximum())
