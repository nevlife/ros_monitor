#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from std_msgs.msg import Bool, UInt32, Float32MultiArray
from sensor_msgs.msg import Image, Imu, NavSatFix, CompressedImage, CameraInfo
from geometry_msgs.msg import TwistWithCovarianceStamped


class MultiTopicSubscriber:
    def __init__(self):

        rospy.init_node("node_connecter", anonymous=True)

        self.status_pub = rospy.Publisher('/node_connecter/status_pub', Bool, queu_size=10) 
        self.data_pub = rospy.Publisher('/node_connecter/node_data', Float32MultiArray, queu_size=10)
        
        
        self.data_init()

        # Subscriber 설정
        rospy.Subscriber("/current_motor_RPM", UInt32, self.callback_motor_rpm)
        rospy.Subscriber("/current_steer", UInt32, self.callback_steer)
        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.callback_gps_fix)
        rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, self.callback_gps_fix_velocity)
        rospy.Subscriber("/imu/data", Imu, self.callback_imu)
        #rospy.Subscriber("/zed_node/left/image_rect_color", Image , self.callback_zed_node_left_image_rect_color)
        rospy.Subscriber("/zed_node/left/camera_info", CameraInfo , self.callback_zed_left_camera_info)
        #rospy.Subscriber("/zed_node/depth/depth_registered", Image , self.callback_zed_node_depth_depth_registered)
        rospy.Subscriber("/zed_node/depth/camera_info", CameraInfo , self.callback_zed_depth_camera_info)
        #rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback_usb_camera)

        rospy.Timer(rospy.Duration(1.0), self.check_missing_data)
        self.rate = rospy.Rate(10)  # 10Hz

    def check_missing_data(self, event):
        if self.rpm is None:
            rospy.logwarn('[RPM Data] No data received from /current_motor_RPM topic.')
                
        if self.steer is None:
            rospy.logwarn('[Steer Data] No data received from /current_steer topic.')
                
        if self.gps_fix is None:
            rospy.logwarn('[GPS Fix] No data received from /ublox_gps/fix topic.')
                
        if self.gps_fix_velocity is None:
            rospy.logwarn('[GPS Velocity] No data received from /ublox_gps/fix_velocity topic.')
                
        if self.imu is None:
            rospy.logwarn('[IMU Data] No data received from /imu/data topic.')

        if self.zed_left_camera_info is None:
            rospy.logwarn('[ZED Left Camera Info] No data received from /zed_node/left/camera_info topic.')
                
        if self.zed_depth_camera_info is None:
            rospy.logwarn('[ZED Depth Camera Info] No data received from /zed_node/depth/camera_info topic.')


    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

        
    '''callback fuc'''
    def callback_motor_rpm(self, msg):
        self.rpm = msg.data

    def callback_steer(self, msg):
        self.steer = msg.data

    def callback_gps_fix(self, msg):
        self.gps_fix['status']['status'] = msg.status.status
        self.gps_fix['status']['service'] = msg.status.service

        self.gps_fix['header']['seq'] = msg.header.seq
        self.gps_fix['header']['stamp']['secs'] = msg.header.stamp.secs
        self.gps_fix['header']['stamp']['nsecs'] = msg.header.stamp.nsecs
        self.gps_fix['header']['frame_id'] = msg.header.frame_id

        self.gps_fix['latitude'] = msg.latitude
        self.gps_fix['longitude'] = msg.longitude
        self.gps_fix['altitude'] = msg.altitude

        self.gps_fix['position_covariance'] = msg.position_covariance
        self.gps_fix['position_covariance_type'] = msg.position_covariance_type
    
    def callback_gps_fix_velocity(self, msg):
        self.gps_fix_velocity['header']['seq'] = msg.header.seq
        self.gps_fix_velocity['header']['stamp']['secs'] = msg.header.stamp.secs
        self.gps_fix_velocity['header']['stamp']['nsecs'] = msg.header.stamp.nsecs
        self.gps_fix_velocity['header']['frame_id'] = msg.header.frame_id

        self.gps_fix_velocity['linear']['x'] = msg.twist.twist.linear.x
        self.gps_fix_velocity['linear']['y'] = msg.twist.twist.linear.y
        self.gps_fix_velocity['linear']['z'] = msg.twist.twist.linear.z

        self.gps_fix_velocity['angular']['x'] = msg.twist.twist.angular.x
        self.gps_fix_velocity['angular']['y'] = msg.twist.twist.angular.y
        self.gps_fix_velocity['angular']['z'] = msg.twist.twist.angular.z

        self.gps_fix_velocity['covariance'] = msg.twist.covariance

    def callback_imu(self, msg):
        self.imu['header']['seq'] = msg.header.seq
        self.imu['header']['stamp']['secs'] = msg.header.stamp.secs
        self.imu['header']['stamp']['nsecs'] = msg.header.stamp.nsecs
        self.imu['header']['frame_id'] = msg.header.frame_id

        self.imu['orientation']['x'] = msg.orientation.x
        self.imu['orientation']['y'] = msg.orientation.y
        self.imu['orientation']['z'] = msg.orientation.z
        self.imu['orientation']['w'] = msg.orientation.w

        self.imu['orientation_covariance'] = msg.orientation_covariance

        self.imu['angular_velocity']['x'] = msg.angular_velocity.x
        self.imu['angular_velocity']['y'] = msg.angular_velocity.y
        self.imu['angular_velocity']['z'] = msg.angular_velocity.z

        self.imu['angular_velocity_covariance'] = msg.angular_velocity_covariance

        self.imu['linear_acceleration']['x'] = msg.linear_acceleration.x
        self.imu['linear_acceleration']['y'] = msg.linear_acceleration.y
        self.imu['linear_acceleration']['z'] = msg.linear_acceleration.z

        self.imu['linear_acceleration_covariance'] = msg.linear_acceleration_covariance


    def callback_zed_depth_camera_info(self, msg):
        self.zed_depth_camera_info['header']['seq'] = msg.header.seq
        self.zed_depth_camera_info['header']['stamp']['secs'] = msg.header.stamp.secs
        self.zed_depth_camera_info['header']['stamp']['nsecs'] = msg.header.stamp.nsecs
        self.zed_depth_camera_info['header']['frame_id'] = msg.header.frame_id

        self.zed_depth_camera_info['height'] = msg.height
        self.zed_depth_camera_info['width'] = msg.width
        self.zed_depth_camera_info['distortion_model'] = msg.distortion_model
        self.zed_depth_camera_info['D'] = msg.D
        self.zed_depth_camera_info['K'] = msg.K
        self.zed_depth_camera_info['R'] = msg.R
        self.zed_depth_camera_info['P'] = msg.P
        self.zed_depth_camera_info['binning_x'] = msg.binning_x
        self.zed_depth_camera_info['binning_y'] = msg.binning_y

        self.zed_depth_camera_info['roi']['x_offset'] = msg.roi.x_offset
        self.zed_depth_camera_info['roi']['y_offset'] = msg.roi.y_offset
        self.zed_depth_camera_info['roi']['height'] = msg.roi.height
        self.zed_depth_camera_info['roi']['width'] = msg.roi.width
        self.zed_depth_camera_info['roi']['do_rectify'] = msg.roi.do_rectify
        
    def callback_zed_left_camera_info(self, msg):
        self.zed_left_camera_info['header']['seq'] = msg.header.seq
        self.zed_left_camera_info['header']['stamp']['secs'] = msg.header.stamp.secs
        self.zed_left_camera_info['header']['stamp']['nsecs'] = msg.header.stamp.nsecs
        self.zed_left_camera_info['header']['frame_id'] = msg.header.frame_id

        self.zed_left_camera_info['height'] = msg.height
        self.zed_left_camera_info['width'] = msg.width
        self.zed_left_camera_info['distortion_model'] = msg.distortion_model
        self.zed_depth_camera_info['D'] = msg.D
        self.zed_depth_camera_info['K'] = msg.K
        self.zed_depth_camera_info['R'] = msg.R
        self.zed_depth_camera_info['P'] = msg.P
        self.zed_left_camera_info['binning_x'] = msg.binning_x
        self.zed_left_camera_info['binning_y'] = msg.binning_y

        self.zed_left_camera_info['roi']['x_offset'] = msg.roi.x_offset
        self.zed_left_camera_info['roi']['y_offset'] = msg.roi.y_offset
        self.zed_left_camera_info['roi']['height'] = msg.roi.height
        self.zed_left_camera_info['roi']['width'] = msg.roi.width
        self.zed_left_camera_info['roi']['do_rectify'] = msg.roi.do_rectify

    def data_init(self):
        self.nodes = ()
        self.rpm = None
        self.steer = None
        self.usb_camera = None
        self.zed_left_camera_img = None
        self.zed_depth_camera_img = None
        
        self.gps_fix = {
            'header': {
                'seq': None,
                'stamp': {'secs': None,'nsecs': None},
                'frame_id': None
            },
            'status': {'status': None,'service': None},
            'latitude': None,
            'longitude': None,
            'altitude': None,
            'position_covariance': [],
            'position_covariance_type': None
        }
        
        self.gps_fix_velocity = {
            'header': {
                'seq': None,
                'stamp': {'secs': None,'nsecs': None},
                'frame_id': None
            },
            'linear': {'x': None,'y': None,'z': None},
            'angular': {'x': None,'y': None,'z': None},
            'covariance': []
        }
        
        self.imu = {
            'header': {
                    'seq': None,
                    'stamp': {'secs': None,'nsecs': None},
                    'frame_id': None
            },
            'orientation': {'x': None,'y': None,'z': None,'w': None},
            'orientation_covariance': [],
            'angular_velocity': {'x': None,'y': None,'z': None},
            'angular_velocity_covariance': [],
            'linear_acceleration': {'x': None,'y': None,'z': None},
            'linear_acceleration_covariance': []
        }
        
        self.zed_depth_camera_info = {
            'header': {
                'seq': None,
                'stamp': {'secs': None,'nsecs': None},
                'frame_id': None
            },
            'height': None,
            'width': None,
            'distortion_model': None,
            'D': [],'K': [],'R': [],'P': [],
            'binning_x': None,
            'binning_y': None,
            'roi': {
                'x_offset': None,
                'y_offset': None,
                'height': None,
                'width': None,
                'do_rectify': None,
            }
        }
        
        self.zed_left_camera_info = {
            'header': {
                'seq': None,
                'stamp': {'secs': None,'nsecs': None},
                'frame_id': None
            },
            'height': None,
            'width': None,
            'distortion_model': None,'D': [],'K': [],'R': [],'P': [],
            'binning_x': None,
            'binning_y': None,
            'roi': {
                'x_offset': None,
                'y_offset': None,
                'height': None,
                'width': None,
                'do_rectify': None
            }
        }
        
if __name__ == "__main__":
    subscriber = MultiTopicSubscriber()
    subscriber.spin()
