#!/usr/bin/env python3
# filepath: /home/pgw/catkin_ws/src/ros_monitor/scripts/test.py

import rospy
from ros_monitor.msg import MonitoringArray

def monitoring_callback(msg):
    """Print MonitoringArray message directly to CLI"""
    print("\n" + "=" * 80)
    print(f"Timestamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
    print("=" * 80)
    
    for info_idx, info in enumerate(msg.info):
        print(f"\n[INFO {info_idx}] Name: {info.name}, Description: {info.description}")
        
        # Print PC information if available
        if hasattr(info, 'pc') and hasattr(info.pc, 'Hostname'):
            print(f"Host: {info.pc.Hostname}, IP: {info.pc.ip}")
        
        print(f"\nTotal {len(info.values)} values:")
        for val_idx, val in enumerate(info.values):
            print(f"  {val_idx:3d}. Key: {val.key:40s} | Value: {val.value:15s} | Unit: {val.unit:10s} | Error Level: {val.errorlevel}")
    
    print("\n" + "=" * 80)

def main():
    rospy.init_node('monitoring_cli', anonymous=True)
    rospy.Subscriber('/monitoring', MonitoringArray, monitoring_callback)
    print("Subscribing to '/monitoring' topic...")
    print("Press Ctrl+C to exit")
    rospy.spin()

if __name__ == '__main__':
    main()