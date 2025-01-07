#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy

def main():
    rospy.init_node('sys_status_monitor')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass