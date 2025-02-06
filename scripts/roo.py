import sys
import os

# 원하는 루트 디렉토리 설정
root_dir = os.path.expanduser("~/catkin_ws/src/vehicle_diag")

# sys.path에 추가
sys.path.append(root_dir)

# 현재 작업 디렉토리 변경 (필요한 경우)
os.chdir(root_dir)

print("Current working directory:", os.getcwd())  # 확인용

