#!/usr/bin/python3
# -*- coding: utf-8 -*-
import subprocess

def main():
    # 각 터미널에서 명령어 실행
    subprocess.Popen(['gnome-terminal', '--', 'rosrun', 'vehicle_diag', 'gui.py'])
    print('Starting gui.py in a new terminal')

    subprocess.Popen(['gnome-terminal', '--', 'rosrun', 'vehicle_diag', 'sys_status.py'])
    print('Starting sys_status_monitor.py in a new terminal')

    subprocess.Popen(['gnome-terminal', '--', 'rosrun', 'vehicle_diag', 'sensor_status.py'])
    print('Starting sensor_status.py in a new terminal')


if __name__ == '__main__':
    main()
