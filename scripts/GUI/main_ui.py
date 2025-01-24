#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import rospy
import signal
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QTabWidget, QSizePolicy
from PySide6.QtCore import Qt
from status_ui import SystemMonitorGUI


class VehicleDiag(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Monitor')
        self.resize(1920, 1080)
        # 탭 위젯 생성
        tabs = QTabWidget()

        # SystemMonitorGUI 탭 추가
        status_tab = SystemMonitorGUI()
        tabs.addTab(status_tab, 'Status')

        # 레이아웃 설정
        tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vbox = QVBoxLayout()
        vbox.addWidget(tabs)
        self.setLayout(vbox)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
            
def signal_handler(sig, frame):
    QApplication.quit()

def main():
    rospy.init_node('vehicle_diag', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    app = QApplication(sys.argv)
    window = VehicleDiag()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
