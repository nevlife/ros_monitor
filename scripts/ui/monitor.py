#!/usr/bin/env python3
# filepath: /home/pgw/catkin_ws/src/ros_monitor/scripts/ui/ui.py

import json
import atexit
import psutil
from collections import deque

import rospy
import logging
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QApplication, QPlainTextEdit
)
from PySide6.QtGui import QFont, QColor
from PySide6.QtCore import Qt, QTimer, Signal, QObject, Slot
from std_msgs.msg import Float32MultiArray, String

from ros_monitor.msg import MonitoringArray
import ros_tracker

MAX_LOG_LINES = 500

class LogHandler(QObject):
    new_log_signal = Signal(str)
    
    def add_log(self, text):
        self.new_log_signal.emit(text)
        
class RosMonitor(QWidget):
    def __init__(self):
        super().__init__()
        # ROS 노드 초기화
        #rospy.init_node('ros_monitor_ui', anonymous=True)
        self.setWindowTitle('ROS Monitor UI')
        self.resize(1920, 1080)

        # 내부 상태 저장용 딕셔너리
        self.total_resource_data = {}
        self.topic_hzbw_data = {}
        self.nodes_resource_data = {}
        self.gpu_proc_data = {}
        
        self.log_handler = LogHandler()
        self.log_handler.new_log_signal.connect(self._append_log_safe)
        
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
        
        rospy.Subscriber('/monitoring', MonitoringArray, self.monitoring_callback, queue_size=10)

        # 1초마다 UI 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(1000)
        
        #클릭타이머
        self.click_timer = QTimer(self)
        self.click_timer.setSingleShot(True)
        self.click_processing = False

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


    def update_ui(self):
        self.update_labels()
        self.update_topics_table()
        self.update_nodes_table()
        self.update_gpu_table()

    def monitoring_callback(self, msg):
        """MonitoringArray 메시지 처리"""
        try:
            # 각 정보 소스 처리
            for info in msg.info:
                monitor_type = info.type
                
                # MonitoringInfo의 type 필드를 기반으로 데이터 분류
                if monitor_type == "topic_hzbw":
                    for value in info.values:
                        # hz_/topic/name 또는 topic_name_hz 형식 처리
                        if value.key.startswith("hz_"):
                            topic_name = value.key[3:]  # "hz_" 접두사 제거
                            
                            if topic_name not in self.topic_hzbw_data:
                                self.topic_hzbw_data[topic_name] = {}
                            
                            # Hz 값과 오류 수준 저장
                            self.topic_hzbw_data[topic_name]['hz'] = float(value.value) if value.value else 0
                            self.topic_hzbw_data[topic_name]['hz_target'] = value.target
                            self.topic_hzbw_data[topic_name]['hz_errorlevel'] = value.errorlevel
                            
                            # 오류 수준이 높은 경우 로그 기록
                            # if value.errorlevel :
                            #     self.append_log(f"[ERROR] Topic {topic_name}: frequency too low - {float(value.errorlevel):.2f} Hz")
                            # elif value.errorlevel >= 1.0:
                            #     self.append_log(f"[WARNING] Topic {topic_name}: frequency below target - {float(value.errorlevel):.2f} Hz")
                        
                        # bw_/topic/name 또는 topic_name_bw 형식 처리
                        elif value.key.startswith("bw_"):
                            topic_name = value.key[3:]  # "bw_" 접두사 제거
                            
                            if topic_name not in self.topic_hzbw_data:
                                self.topic_hzbw_data[topic_name] = {}
                            
                            # 대역폭 값과 함께 단위도 저장
                            self.topic_hzbw_data[topic_name]['bw'] = float(value.value) if value.value else 0
                            self.topic_hzbw_data[topic_name]['bw_unit'] = value.unit  # 단위 정보 저장
                            
                elif monitor_type == "nodes_resource":
                     for value in info.values:
                        if value.key.startswith("cpu_"):
                            node_name = value.key.replace("cpu_", "")

                            if node_name not in self.nodes_resource_data:
                                self.nodes_resource_data[node_name] = {}
                            cpu = value.value
                            self.nodes_resource_data[node_name]['cpu'] = value.value
                            
                            # CPU 사용량이 높은 경우 로그 기록
                            if cpu > 90:
                                self.append_log(f"[ERROR] Node {node_name}: high CPU usage - {cpu:.2f}%")
                            elif cpu > 80:
                                self.append_log(f"[WARNING] Node {node_name}: elevated CPU usage - {cpu:.2f}%")
                        
                        elif value.key.startswith("mem_"):
                            node_name = value.key.replace("mem_", "")
                            
                            if node_name not in self.nodes_resource_data:
                                self.nodes_resource_data[node_name] = {}

                            self.nodes_resource_data[node_name]['mem'] = value.value
                elif monitor_type == "system_resource":
                     # 처리할 키 목록 정의
                    system_keys = [
                        "cpu_usage", "cpu_temp", 
                        "load_avg_1min", "load_avg_5min", "load_avg_15min", 
                        "mem_used", "mem_total", "mem_usage_percent",
                        "gpu_usage", "gpu_mem_used", "gpu_mem_total", "gpu_mem_usage_percent", "gpu_temp"
                    ]
                    
                    for value in info.values:
                        # 해당 키가 목록에 있으면 데이터 저장
                        if value.key in system_keys:
            
                            # 값, 단위 및 오류 수준 저장 
                            self.total_resource_data[value.key] = {
                                'value': float(value.value) if value.value else 0,
                                'unit': value.unit,
                                'errorlevel': value.errorlevel,
                                'target': value.target
                            }
                            
                else:
                    pass
                
        except Exception as e:
            rospy.logerr(f"[ui] Error processing monitoring data: {str(e)}")
            self.append_log(f"Error processing monitoring data: {str(e)}")

    def update_labels(self):
        """헤더 라벨 업데이트"""
        g = self.total_resource_data.get
        
        # 각 항목의 값을 딕셔너리에서 추출
        cpu_usage = g('cpu_usage', {}).get('value', -1)
        cpu_temp = g('cpu_temp', {}).get('value', -1)
        load1 = g('load_avg_1min', {}).get('value', -1)
        load5 = g('load_avg_5min', {}).get('value', -1)
        load15 = g('load_avg_15min', {}).get('value', -1)
        
        # 에러 레벨도 가져와서 필요시 사용 가능
        cpu_usage_error = g('cpu_usage', {}).get('errorlevel', 0)
        
        mem_used = g('mem_used', {}).get('value', -1)
        mem_total = g('mem_total', {}).get('value', -1)
        mem_percent = g('mem_usage_percent', {}).get('value', -1)
        
        # 단위 정보도 가져옴
        mem_used_unit = g('mem_used', {}).get('unit', 'MB')
        
        gpu_usage = g('gpu_usage', {}).get('value', -1)
        gpu_mem_used = g('gpu_mem_used', {}).get('value', -1)
        gpu_mem_total = g('gpu_mem_total', {}).get('value', -1)
        gpu_temp = g('gpu_temp', {}).get('value', -1)

        # 에러 레벨에 따라 색상 변경 등의 시각화도 가능
        cpu_style = "color: red;" if cpu_usage_error > 1.0 else ""

        self.cpu_label.setText(
            f'CPU: {cpu_usage:.2f}% | Temp: {cpu_temp:.1f}C | Load: {load1:.2f}, {load5:.2f}, {load15:.2f}'
        )
        if cpu_usage_error > 1.0:
            self.cpu_label.setStyleSheet("color: red;")
        
        self.mem_label.setText(
            f'Mem: {mem_used:.1f}/{mem_total:.1f} {mem_used_unit} ({mem_percent:.2f}%)'
        )
        
        self.gpu_label.setText(
            f'GPU: {gpu_usage:.2f}% | Mem: {gpu_mem_used:.1f}/{gpu_mem_total:.1f} MB | Temp: {gpu_temp:.1f}C'
        )

    def update_topics_table(self):
        """토픽 테이블 업데이트"""
        self.topics_table.setRowCount(len(self.topic_hzbw_data))
        for row, (topic, values) in enumerate(sorted(self.topic_hzbw_data.items())):
            hz_value = values.get('hz', -1)
            bw_value = values.get('bw', -1)
            bw_unit = values.get('bw_unit', 'B/s')  # 단위 정보 가져오기, 기본값은 'B/s'
            target_hz = values.get('hz_target')
            hz_error_level = values.get('hz_errorlevel', -1)
            color = None

            if hz_error_level >= 2.0:
                color = QColor('red')
            elif hz_error_level >= 1.0:
                color = QColor('orange')

            topic_item = QTableWidgetItem(topic)
            hz_item = QTableWidgetItem(f"{hz_value:.2f}")
            bw_item = QTableWidgetItem(f"{bw_value:.2f} {bw_unit}")  # 단위 정보 포함
            
            for item in [topic_item, hz_item, bw_item]:
                item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                if color:
                    item.setForeground(color)
                    item.setFont(self.bold_font)

            self.topics_table.setItem(row, 0, topic_item)
            self.topics_table.setItem(row, 1, hz_item)
            self.topics_table.setItem(row, 2, bw_item)
            
    def update_nodes_table(self):
        """노드 테이블 업데이트"""
        self.nodes_table.setRowCount(len(self.nodes_resource_data))
        for row, (node, resources) in enumerate(sorted(self.nodes_resource_data.items())):
            cpu_value = resources.get('cpu', -1)
            mem_value = resources.get('mem', -1)
            
            node_item = QTableWidgetItem(node)
            cpu_item = QTableWidgetItem(f"{cpu_value:.2f}")
            mem_item = QTableWidgetItem(f"{mem_value:.2f} MB")
            
            # CPU 사용량 경고 설정
            if cpu_value > 90:
                cpu_item.setForeground(QColor('red'))
                cpu_item.setFont(self.bold_font)
            elif cpu_value > 80:
                cpu_item.setForeground(QColor('orange'))
                cpu_item.setFont(self.bold_font)
                
            for item in [node_item, cpu_item, mem_item]:
                item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                
            self.nodes_table.setItem(row, 0, node_item)
            self.nodes_table.setItem(row, 1, cpu_item)
            self.nodes_table.setItem(row, 2, mem_item)

    def update_gpu_table(self):
        """GPU 테이블 업데이트"""
        self.gpu_proc_table.setRowCount(len(self.gpu_proc_data))
        for row, (pid, info) in enumerate(sorted(self.gpu_proc_data.items(), key=lambda x: x[0])):
            pid_item   = QTableWidgetItem(pid)
            type_item  = QTableWidgetItem(info.get('type', '-'))
            sm_item    = QTableWidgetItem(str(info.get('sm', 0)))
            mem_item   = QTableWidgetItem(str(info.get('mem', 0)))
            enc_item   = QTableWidgetItem(str(info.get('enc', 0)))
            dec_item   = QTableWidgetItem(str(info.get('dec', 0)))
            jpg_item   = QTableWidgetItem(str(info.get('jpg', 0)))
            ofa_item   = QTableWidgetItem(str(info.get('ofa', 0)))
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
        if self.click_processing:
            return
        self.click_processing = True
        
        topic_item = self.topics_table.item(row, 0)
        if not topic_item:
            self.click_processing = False
            return
            
        clicked_topic = topic_item.text()
        
        self.append_log("\n\n===================================================")
        self.append_log(f"Topic : \"{clicked_topic}\"")
        
        QApplication.processEvents()
        
        try:
            pubs, subs = ros_tracker.find_topic_connections(clicked_topic)
            
            self.append_log(f"Nodes publishing this topic: {pubs}")
            self.append_log(f"Nodes subscribing to this topic: {subs}")
            QApplication.processEvents()  # UI 갱신

            self.append_log("\nNodes publishing the clicked topic\n")
            if pubs:
                for pub_node in pubs:
                    # 중간에 이벤트 처리 허용
                    QApplication.processEvents()
                    
                    pub_topics, sub_topics = ros_tracker.find_node_connections(pub_node)
                    self.append_log(f"Node : {pub_node}")
                    self.append_log("Publishes :")
                    for topic in pub_topics:
                        self.append_log(f'      {topic}')
                    self.append_log("Subscribes :")
                    for topic in sub_topics:
                        self.append_log(f'      {topic}')
                    self.append_log("\n")
                    
                    # 각 노드 처리 후 이벤트 처리
                    QApplication.processEvents()
            else:
                self.append_log("Not Found")

            self.append_log("\nNodes subscribing to the clicked topic\n")
            if subs:
                for sub_node in subs:
                    # 중간에 이벤트 처리 허용
                    QApplication.processEvents()
                    
                    pub_topics, sub_topics = ros_tracker.find_node_connections(sub_node)
                    self.append_log(f"Node : {sub_node}")
                    self.append_log("Publishes :")
                    for topic in pub_topics:
                        self.append_log(f'      {topic}')
                    self.append_log("Subscribes :")
                    for topic in sub_topics:
                        self.append_log(f'      {topic}')
                    self.append_log("\n")
                    
                    # 각 노드 처리 후 이벤트 처리
                    QApplication.processEvents()
            else:
                self.append_log("Not Found")
        except Exception as e:
            self.append_log(f"오류 발생: {str(e)}")
        finally:
            self.click_processing = False
        
    def node_clicked(self, row, col):
        if self.click_processing:
            return
        self.click_processing = True
        
        try:
            node_item = self.nodes_table.item(row, 0)
            if not node_item:
                return
            node_name = node_item.text()
            
            # 클릭 처리 준비 메시지 표시
            self.append_log(f'\n\n===================================================')
            self.append_log(f'Node Clicked: "{node_name}" - 정보 로딩 중...')
            
            # 이벤트 처리를 위한 호출 - UI 갱신 허용
            QApplication.processEvents()
            
            # 무거운 ROS 작업 실행
            pub_topics, sub_topics = ros_tracker.find_node_connections(node_name)
            
            self.append_log('Publishes topic:')
            for topic in pub_topics:
                self.append_log(f'   {topic}')
            self.append_log('Subscribes topic:')
            for topic in sub_topics:
                self.append_log(f'   {topic}')
            
            QApplication.processEvents()  # UI 갱신
            
            pid, cmdline = ros_tracker.find_node_pid(node_name)
            if pid:
                self.append_log(f'PID: {pid}')
                self.append_log(f'Command: {cmdline}')
            else:
                self.append_log('PID: Not Found')
        except Exception as e:
            self.append_log(f"오류 발생: {str(e)}")
        finally:
            self.click_processing = False
        
    def append_log(self, text: str):
            """스레드 안전하게 로그를 추가하는 메서드"""
            # 로그 핸들러를 통해 메인 스레드로 신호 전달
            self.log_handler.add_log(text)

    @Slot(str)
    def _append_log_safe(self, text: str):
        """메인 스레드에서 실행되는 실제 로그 추가 메서드"""
        scroll_bar = self.log_widget.verticalScrollBar()
        at_bottom = scroll_bar.value() == scroll_bar.maximum()
        self.log_lines.append(text)
        if len(self.log_lines) == MAX_LOG_LINES:
            self.log_widget.setPlainText('\n'.join(self.log_lines))
        else:
            self.log_widget.appendPlainText(text)
        if at_bottom:
            self.log_widget.verticalScrollBar().setValue(self.log_widget.verticalScrollBar().maximum())

import sys
import threading
import signal
import atexit
import time
    
def main():
    
    def signal_handler(sig, frame):
        print("Signal received, shutting down...")
        QApplication.quit()
    
    def cleanup(window):
        """애플리케이션 종료 시 정리 작업 수행"""
        print("Cleaning up before exit...")
        
        # UI 리소스 정리
        if hasattr(window, 'timer') and window.timer.isActive():
            window.timer.stop()
        
        # 클래스 인스턴스의 참조 해제
        window.deleteLater()
        
        # ROS 종료
        safe_shutdown()
        
        print("Cleanup complete")
        
    def safe_shutdown():
        """ROS 노드 안전 종료"""
        try:
            if not rospy.is_shutdown():
                print("Shutting down ROS node...")
                rospy.signal_shutdown('Application closed')
                
                # 종료 메시지가 전달될 때까지 짧게 대기
                timeout = time.time() + 1.0  # 1초 타임아웃
                while not rospy.is_shutdown() and time.time() < timeout:
                    time.sleep(0.1)
        except Exception as e:
            print(f"Error during ROS shutdown: {e}")
            
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
        
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)

    window = RosMonitor()
    window.show()
        
    app.aboutToQuit.connect(lambda: cleanup(window))


    ros_thread = threading.Thread(target=rospy.spin)
    ros_thread.daemon = True
    ros_thread.start()
    
    atexit.register(lambda: safe_shutdown())
    
    try:
        sys.exit(app.exec())
    except SystemExit:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
        safe_shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()