#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image, Imu, NavSatFix, CameraInfo
from geometry_msgs.msg import TwistStamped
from zed_interfaces.msg import ObjectsStamped

class MultiTopicSubscriber:
    def __init__(self):
        """
        ROS 노드 초기화 및 여러 토픽 구독 설정.
        """
        rospy.init_node("multi_topic_subscriber", anonymous=True)

        # 데이터 저장용 변수
        self.motor_rpm = None
        self.steer = None
        self.imu_data = None
        self.gps_fix = None
        self.gps_velocity = None
        self.navpvt = None
        self.camera_image = None
        self.depth_info = None
        self.depth_image = None
        self.left_info = None
        self.left_image = None
        self.objects = None

        # Subscriber 설정
        rospy.Subscriber("/current_motor_RPM", Float32, self.callback_motor_rpm)
        rospy.Subscriber("/current_steer", Float32, self.callback_steer)
        rospy.Subscriber("/imu/data", Imu, self.callback_imu)
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.callback_gps_fix)
        rospy.Subscriber("/ublox_gps/fix_velocity", TwistStamped, self.callback_gps_velocity)
        rospy.Subscriber("/ublox_gps/navpvt", String, self.callback_navpvt)
        rospy.Subscriber("/usb_cam/image_raw/compressed", Image, self.callback_camera_image)
        rospy.Subscriber("/zed_node/depth/camera_info", CameraInfo, self.callback_depth_info)
        rospy.Subscriber("/zed_node/depth/depth_registered", Image, self.callback_depth_image)
        rospy.Subscriber("/zed_node/left/camera_info", CameraInfo, self.callback_left_info)
        rospy.Subscriber("/zed_node/left/image_rect_color", Image, self.callback_left_image)
        rospy.Subscriber("/zed_node/obj_det/objects", ObjectsStamped, self.callback_objects)

    # 콜백 함수 정의
    def callback_motor_rpm(self, msg):
        self.motor_rpm = msg.data
        rospy.loginfo(f"Motor RPM: {self.motor_rpm}")

    def callback_steer(self, msg):
        self.steer = msg.data
        rospy.loginfo(f"Steer: {self.steer}")

    def callback_imu(self, msg):
        self.imu_data = msg
        rospy.loginfo(f"IMU Data Received")

    def callback_gps_fix(self, msg):
        self.gps_fix = msg
        rospy.loginfo(f"GPS Fix Received: {self.gps_fix.latitude}, {self.gps_fix.longitude}")

    def callback_gps_velocity(self, msg):
        self.gps_velocity = msg
        rospy.loginfo(f"GPS Velocity: {self.gps_velocity.twist.linear}")

    def callback_navpvt(self, msg):
        self.navpvt = msg.data
        rospy.loginfo(f"NavPVT: {self.navpvt}")

    def callback_camera_image(self, msg):
        self.camera_image = msg
        rospy.loginfo(f"Camera Image Received")

    def callback_depth_info(self, msg):
        self.depth_info = msg
        rospy.loginfo(f"Depth Camera Info Received")

    def callback_depth_image(self, msg):
        self.depth_image = msg
        rospy.loginfo(f"Depth Image Received")

    def callback_left_info(self, msg):
        self.left_info = msg
        rospy.loginfo(f"Left Camera Info Received")

    def callback_left_image(self, msg):
        self.left_image = msg
        rospy.loginfo(f"Left Camera Image Received")

    def callback_objects(self, msg):
        self.objects = msg
        rospy.loginfo(f"Detected Objects Received")

    def spin(self):
        """
        ROS Spin을 실행하여 콜백 함수가 호출되도록 유지.
        """
        rospy.spin()

def main():
    subscriber = MultiTopicSubscriber()
    subscriber.spin()
    
if __name__ == "__main__":
    main()