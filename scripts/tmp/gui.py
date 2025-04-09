#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
import signal
import rospy
import psutil
import json
import html

import atexit
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QApplication,
    QPlainTextEdit,
)
from PySide6.QtGui import QFont, QColor, QTextCursor

from PySide6.QtCore import Qt, QTimer
from std_msgs.msg import Float32MultiArray, String

from collections import deque

# ros_topic_tracker.py에 find_topic_connections, find_node_connections가 정의되어 있다고 가정
import ros_tracker

MAX_LOG_LINES = 500
def signal_handler(sig, frame):
    QApplication.quit()

class RosMonitor(QWidget):
    def __init__(self):
        super().__init__()

        # ROS 노드 초기화
        rospy.init_node('ros_monitor_ui', anonymous=True)

        self.setWindowTitle('ROS Monitor UI')
        self.resize(1920, 1080)

        # 내부 상태 보관용 딕셔너리
        self.total_resource_data = {}
        self.topic_hzbw_data = {}
        self.nodes_resource_data = {}
        self.gpu_proc_data = {}

        # CPU/RAM/GPU 라벨
        self.cpu_label = QLabel('CPU Info: -')
        self.mem_label = QLabel('Memory Info: -')
        self.gpu_label = QLabel('GPU Info: -')

        self.log_lines = deque(maxlen=MAX_LOG_LINES)  # deque 사용
        self.log_widget = QPlainTextEdit()            # PlainTextEdit으로 변경
        self.log_widget.setReadOnly(True)

        
        self.bold_f = QFont()
        self.bold_f.setBold(True)
        
        self.init_ui()
        rospy.Subscriber('/total_resource', Float32MultiArray, self.total_resource_callback)
        rospy.Subscriber('/topics_hzbw', String, self.topic_hzbw_callback)
        rospy.Subscriber('/nodes_resource', String, self.nodes_resource_callback)
        rospy.Subscriber('/gpu_pmon', String, self.gpu_pmon_callback)

        # 1초마다 UI 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(1000)

    def update_ui(self):
        self.update_labels()
        self.update_topics_table()
        self.update_nodes_table()
        self.update_gpu_table()

    def init_ui(self):
        logical_cores = psutil.cpu_count(logical=True)
        self.layout = QGridLayout(self)

        self.layout.addWidget(self.log_widget, 0, 0, 10, 1)


        label_layout = QVBoxLayout()
        label_layout.addWidget(self.cpu_label)
        label_layout.addWidget(self.mem_label)
        label_layout.addWidget(self.gpu_label)
        self.layout.addLayout(label_layout, 0, 1, 1, 2)


        self.topics_table = QTableWidget(0, 3)
        self.topics_table.setHorizontalHeaderLabels(['Topic name', 'Hz', 'Bandwidth'])
        self.topics_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.topics_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.topics_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.layout.addWidget(self.topics_table, 1, 1, 3, 1)
        self.topics_table.cellClicked.connect(self.topic_clicked)


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


        self.gpu_proc_table = QTableWidget(0, 9)
        self.gpu_proc_table.setHorizontalHeaderLabels([
            'pid', 'type', 'sm', 'mem', 'enc', 'dec', 'jpg', 'ofa', 'command'
        ])
        for i in range(8):
            self.gpu_proc_table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeToContents)
        self.gpu_proc_table.horizontalHeader().setSectionResizeMode(8, QHeaderView.Stretch)
        self.layout.addWidget(self.gpu_proc_table, 4, 1, 3, 2)


        self.setLayout(self.layout)


    def total_resource_callback(self, msg):
        f = [
            'cpu_user', 'cpu_nice', 'cpu_system', 'cpu_idle', 'cpu_iowait',
            'cpu_irq', 'cpu_softirq', 'cpu_steal', 'cpu_guest', 'cpu_guest_nice',
            'cpu_usage_percent', 'cpu_temp',
            'cpu_load_1min', 'cpu_load_5min', 'cpu_load_15min',
            'mem_used', 'mem_total', 'mem_usage_percent',
            'gpu_usage_percent', 'gpu_mem_used', 'gpu_mem_total', 'gpu_mem_usage', 'gpu_temp'
        ]
        self.total_resource_data = {n: v for n, v in zip(f, msg.data)}

    def topic_hzbw_callback(self, msg):
        tmp = {}
        arr = json.loads(msg.data)
        for i in arr:
            tmp[i['topic']] = {
                'hz': i['hz'],
                'bw': i['bw'],
                'target_hz': i['target_hz'],
            }
        self.topic_hzbw_data = tmp

    def nodes_resource_callback(self, msg):
        tmp = {}
        arr = json.loads(msg.data)
        for i in arr:
            tmp[i['node']] = {
                'cpu': i['cpu'],
                'mem': i['mem']
            }
        self.nodes_resource_data = tmp

    def gpu_pmon_callback(self, msg):
        tmp = {}
        arr = json.loads(msg.data)
        for i in arr:
            tmp[i['pid']] = {
                'gpu_idx': i['gpu_idx'],
                'type': i['type'],
                'sm': i['sm'],
                'mem': i['mem'],
                'enc': i['enc'],
                'dec': i['dec'],
                'jpg': i['jpg'],
                'ofa': i['ofa'],
                'command': i['command'],
            }
        self.gpu_proc_data = tmp


    def update_labels(self):
        g = self.total_resource_data.get
        cpu_usage = g('cpu_usage_percent', -1)
        cpu_temp = g('cpu_temp', -1)
        load_1 = g('cpu_load_1min', -1)
        load_5 = g('cpu_load_5min', -1)
        load_15 = g('cpu_load_15min', -1)

        mem_percent = g('mem_usage_percent', -1)
        mem_used = g('mem_used', -1)
        mem_total = g('mem_total', -1)

        gpu_usage = g('gpu_usage_percent', -1)
        gpu_mem_used = g('gpu_mem_used', -1)
        gpu_mem_total = g('gpu_mem_total', -1)
        gpu_temp = g('gpu_temp', -1)

        self.cpu_label.setText(
            f'CPU: {cpu_usage:.2f}% | Temp: {cpu_temp:.1f}C | Load: {load_1:.2f}, {load_5:.2f}, {load_15:.2f}'
        )
        self.mem_label.setText(
            f'Mem: {mem_used:.1f}/{mem_total:.1f} MB ({mem_percent:.2f}%)'
        )
        self.gpu_label.setText(
            f'GPU: {gpu_usage:.2f}% | Mem: {gpu_mem_used:.1f}/{gpu_mem_total:.1f} MB | Temp: {gpu_temp:.1f}C'
        )

    def update_topics_table(self):
        self.topics_table.setRowCount(len(self.topic_hzbw_data))
        for row_idx, (t_n, v_dict) in enumerate(self.topic_hzbw_data.items()):
            hz_v = v_dict.get('hz', -1) #int
            bw_v = v_dict.get('bw', '0 B/s') #str
            target_hz = v_dict.get('target_hz', None) #str
            
            c = None
            
            if target_hz is not None:
                target_hz = float(target_hz)
                ratio = hz_v /  target_hz if target_hz != 0 else 9999
                if ratio < 0.6:
                    rospy.logerr(f'[Hz Error] {t_n}: {hz_v} Hz / 60% of target hz ({target_hz})')
                    self.append_log(f'[Hz Error] {t_n}: {hz_v} Hz / 60% of target hz ({target_hz})')
                    c = QColor('red')
                elif ratio < 0.8:
                    rospy.logwarn(f'[Hz Warn] {t_n}: {hz_v} Hz / 80% of target hz ({target_hz})')
                    self.append_log(f'[Hz Warn] {t_n}: {hz_v} Hz / 80% of target hz ({target_hz})')
                    c = QColor('orange')
                    
            t_i = QTableWidgetItem(t_n)
            hz_i = QTableWidgetItem(str(hz_v))
            bw_i = QTableWidgetItem(str(bw_v))

            for it in [t_i, hz_i, bw_i]:
                it.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                if c:
                    it.setForeground(c)
                    it.setFont(self.bold_f)

                
            self.topics_table.setItem(row_idx, 0, t_i)
            self.topics_table.setItem(row_idx, 1, hz_i)
            self.topics_table.setItem(row_idx, 2, bw_i)

    def update_nodes_table(self):
        self.nodes_table.setRowCount(len(self.nodes_resource_data))
        for row_idx, (n_n, r) in enumerate(self.nodes_resource_data.items()):
            cpu_v = r.get('cpu', -1)
            mem_v = r.get('mem', 'N/A')

            n_i = QTableWidgetItem(n_n)
            cpu_i = QTableWidgetItem(f'{cpu_v:.2f}')
            mem_i = QTableWidgetItem(str(mem_v))

            for it in [n_i, cpu_i, mem_i]:
                it.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

            self.nodes_table.setItem(row_idx, 0, n_i)
            self.nodes_table.setItem(row_idx, 1, cpu_i)
            self.nodes_table.setItem(row_idx, 2, mem_i)

    def update_gpu_table(self):
        self.gpu_proc_table.setRowCount(len(self.gpu_proc_data))
        for row_idx, (pid, info) in enumerate(self.gpu_proc_data.items()):
            type_ = info.get('type', '-')
            sm_ = info.get('sm', -1)
            mem_ = info.get('mem', -1)
            enc_ = info.get('enc', -1)
            dec_ = info.get('dec', -1)
            jpg_ = info.get('jpg', -1)
            ofa_ = info.get('ofa', -1)
            cmd_ = info.get('command', '-')

            pid_i = QTableWidgetItem(pid)
            type_i = QTableWidgetItem(type_)
            sm_i = QTableWidgetItem(f'{sm_}')
            mem_i = QTableWidgetItem(f'{mem_}')
            enc_i = QTableWidgetItem(f'{enc_}')
            dec_i = QTableWidgetItem(f'{dec_}')
            jpg_i = QTableWidgetItem(f'{jpg_}')
            ofa_i = QTableWidgetItem(f'{ofa_}')
            cmd_i = QTableWidgetItem(cmd_)

            for it in [pid_i, type_i, sm_i, mem_item, enc_item, dec_i, _item, ofa_i, cmd_item]:
                it.setFlags(Qt.ItemIsScte | Qt.ItemIabled)

            self.gpu_proc_table.setItem(row_idx, 0, pid_i)
            self.gpu_proc_table.setItem(row_idx, 1, type_i)
            self.gpu_proc_table.setItem(row_idx, 2, sm_i)
            self.gpu_proc_table.setItem(row_idx, 3, mem_i)
            self.gpu_proc_table.setItem(row_idx, 4, enc_i)
            self.gpu_proc_table.setItem(row_idx, 5, dec_i)
            self.gpu_proc_table.setItem(row_idx, 6, jpg_i)
            self.gpu_proc_table.setItem(row_idx, 7, ofa_i)
            self.gpu_proc_table.setItem(row_idx, 8, cmd_i)

    def update_topic_list(self):
        master = rospy.get_master()
        state = master.getSystemState()
        pubs = state[0]  # [(topic_name, [node1, node2...]), ...]
        # subs = state[1]
        # etc.

        all_topics = sorted([t for t, _ in pubs])
        self.topics_table.setRowCount(len(all_topics))
        for i, topic_name in enumerate(all_topics):
            item = QTableWidgetItem(topic_name)
            item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.topics_table.setItem(i, 0, item)


    def topic_clicked(self, row, col):

        topic_i = self.topics_table.item(row, 0)
        
        if not topic_i:
            return

        clicked_topic = topic_i.text()
        
        pubs, subs = ros_tracker.find_topic_connections(clicked_topic)

        self.append_log(f"===================================================")
        self.append_log(f"Topic : \"{clicked_topic}\"")
        self.append_log(f"Nodes publishing this topic: {pubs}") #클릭한 토픽을 발행하는 노드
        self.append_log(f"Nodes subscribing to this topic: {subs}") #클릭한 토픽을 구독하는 노드드

        #클릭한 토픽을 발행하는 노드들의 정보
        self.append_log("Nodes publishing the clicked topic")
        if len(pubs) != 0:
            for p_node in pubs:
                pub_t, sub_t = ros_tracker.find_node_connections(p_node)
                self.append_log(f"Node : {p_node}")
                self.append_log(f"Publishes :")
                for i in pub_t:
                    self.append_log(f'      {i}')
                self.append_log(f"Subscribes :")
                for i in sub_t:
                    self.append_log(f'      {i}')
                self.append_log("")
        else:
            self.append_log("Not Found")

        #클릭한 토픽을 구독하는 노드의 정보
        self.append_log("Nodes subscribing to the clicked topic")
        
        if len(subs) != 0:
            for s_node in subs:
                pub_t, sub_t = ros_tracker.find_node_connections(s_node)
                self.append_log(f"Node : {s_node}")
                self.append_log(f"Publishes :")
                for t in pub_t:
                    self.append_log(f'      {t}')
                self.append_log(f"Subscribes :")
                for t in sub_t:
                    self.append_log(f'      {t}')
                self.append_log("")
        else:
            self.append_log("Not Found")
            
    def node_clicked(self, row, col):
        node_i = self.nodes_table.item(row, 0)
        if not node_i:
            return

        n_n = node_i.text()
        self.append_log(f'===================================================')
        self.append_log(f'Node Clicked: "{n_n}"')

        # 노드의 pub/sub 토픽
        pub_t, sub_t = ros_tracker.find_node_connections(n_n)
        self.append_log(f'Publishes topic:')
        for t in pub_t:
            self.append_log(f'   {t}')
        self.append_log(f'Subscribes topic:')
        for t in sub_t:
            self.append_log(f'   {t}')

        # PID와 실행 명령어
        pid, cmdline = ros_tracker.find_node_pid(n_n)
        if pid:
            self.append_log(f'PID: {pid}')
            self.append_log(f'Command: {cmdline}')
        else:
            self.append_log('PID: Not Found')
            
            
    def append_log(self, text: str):
        scroll_bar = self.log_widget.verticalScrollBar()
        at_bottom = scroll_bar.value() == scroll_bar.maximum()

        self.log_lines.append(text)

        if len(self.log_lines) == MAX_LOG_LINES:
            # 전체 텍스트 다시 그릴 때만 (처음 한 번만 발생)
            self.log_widget.setPlainText('\n'.join(self.log_lines))
        else:
            self.log_widget.appendPlainText(text)

        if at_bottom:
            self.log_widget.verticalScrollBar().setValue(self.log_widget.verticalScrollBar().maximum())





def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    app = QApplication(sys.argv)
    
    window = RosMonitor()
    window.show()

    #ROS spin을 별도 스레드에서 실행
    import threading
    ros_thread = threading.Thread(target=rospy.spin)
    ros_thread.start()

    def cleanup():
        rospy.loginfo("[ui] Cleaning up...")
        try:
            if window:
                window.close()
            rospy.signal_shutdown('UI Closed')
        except Exception as e:
            rospy.logerr("[ui]During cleanup:", e)
            traceback.print_exc()

    atexit.register(cleanup)

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        rospy.loginfo("[ui]Ctrl+C detected, shutting down...")
        cleanup()
    except Exception as e:
        rospy.logerr("[ui]Exception caught in main:")
        traceback.print_exc()
        cleanup()


if __name__ == '__main__':
    main()
