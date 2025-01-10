#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from message_filters import Subscriber

def sys_status_callback(msg):
    """
    콜백 함수: 수신된 데이터를 처리.
    """
    # 수신된 메시지 출력
    rospy.loginfo(f"Received data: {msg.data}")

def main():
    """
    메인 함수: ROS 노드 초기화 및 Subscriber 설정.
    """
    rospy.init_node('listener_node', anonymous=True)

    # Float32MultiArray를 구독하는 Subscriber 생성
    node = Subscriber('/multi_topic_subscriber', Float32MultiArray, sys_status_callback)

    # ROS 이벤트 루프 실행
    rospy.spin()

if __name__ == '__main__':
    main()
