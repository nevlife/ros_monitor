#!/usr/bin/python3
# -*- coding: utf-8 -*-
# filepath: /home/pgw/catkin_ws/src/ros_monitor/scripts/ui/main.py

import sys
import rospy
import signal
import threading
import atexit
import time

from PySide6.QtWidgets import QApplication, QWidget, QTabWidget, QVBoxLayout
from monitor import RosMonitor

class Main(QWidget):
    def __init__(self):
        super().__init__()
        
        self.initUI()

    def initUI(self):
        self.tabs = QTabWidget()

        self.ros_monitor_tab = RosMonitor()
        self.rosbag_tab = QWidget()

        self.tabs.addTab(self.ros_monitor_tab, 'Monitor')
        self.tabs.addTab(self.rosbag_tab, 'Rosbag')

        vbox = QVBoxLayout()
        vbox.addWidget(self.tabs)

        self.setLayout(vbox)

        self.setWindowTitle('ROS Monitor System')
        self.resize(1920, 1080)


def main():
    # ROS 노드 초기화를 가장 먼저 수행
    rospy.init_node('ros_monitor_ui', anonymous=True)
    
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
            
    # SIGINT(Ctrl+C), SIGTERM 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
        
    # Qt 애플리케이션 생성
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)

    # 메인 윈도우 생성
    window = Main()
    
    # 종료 시 정리 작업 연결
    app.aboutToQuit.connect(lambda: cleanup(window))

    # 별도 스레드에서 ROS spin 실행
    ros_thread = threading.Thread(target=rospy.spin)
    ros_thread.daemon = True  # 메인 스레드 종료 시 자동 종료
    ros_thread.start()
    
    # 종료 시 정리 작업 등록
    atexit.register(lambda: safe_shutdown())
    
    # 윈도우 표시
    window.show()
    
    try:
        # Qt 이벤트 루프 실행
        sys.exit(app.exec())
    except SystemExit:
        # 정상 종료
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
        safe_shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()