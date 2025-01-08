from PySide6.QtWidgets import QApplication, QWidget, QLabel, QHBoxLayout
import sys

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()

        # QHBoxLayout을 생성
        layout = QHBoxLayout()

        # 라벨 추가
        label1 = QLabel("Label 1")
        label2 = QLabel("Label 2")
        label3 = QLabel("Label 3")

        layout.addWidget(label1)  # 첫 번째 라벨 추가
        layout.addWidget(label2)  # 두 번째 라벨 추가
        layout.addWidget(label3)  # 세 번째 라벨 추가

        # 레이아웃을 위젯에 설정
        self.setLayout(layout)
        self.setWindowTitle("Horizontal Box Layout Example")
        self.setGeometry(300, 300, 400, 100)

def main():
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
