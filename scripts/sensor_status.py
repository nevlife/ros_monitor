#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random
import rospy
from std_msgs.msg import Bool, Int8MultiArray

def random_bool():
    return random.choice([True, False])
def sensor_status():
    sensor_status_pub  = rospy.Publisher('/sensor_status/sensor_status_bool', Int8MultiArray, queue_size=10)
    
    lidar_status = random_bool()
    gps_status = random_bool()
    camera_status = random_bool()
    
    status_list = [lidar_status, gps_status, camera_status]

    msg = Int8MultiArray()
    msg.data = status_list
    
    print(bool(lidar_status), bool(gps_status), bool(camera_status))
    sensor_status_pub.publish(msg)    

if __name__ == '__main__':
    try:
        rospy.init_node('sensor_status', anonymous=True)
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            sensor_status()
            rate.sleep()
        sensor_status()

    except rospy.ROSInterruptException:
        pass