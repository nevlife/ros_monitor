#!/usr/bin/env python3
# filepath: /home/pgw/catkin_ws/src/ros_monitor/scripts/core/core.py

import socket
import rospy
from ros_monitor.msg import *
from enum import Enum

class AggregationStrategies(Enum):
    LAST = 1
    FIRST = 2
    MIN = 3
    MAX = 4
    AVG = 5
    
class Monitor(object):
    def __init__(self, monitorDescription, autoPublishing=True, monitoring_type="generic"):
        rospy.loginfo(f"Monitor initialized with description: {monitorDescription}")
        self.ma = MonitoringArray()
        self.description = monitorDescription
        self.monitoring_type = monitoring_type
        mi = MonitoringInfo()
        self.host_name = socket.gethostname()
        self.node_name = rospy.get_name()
        mi.name = self.host_name + self.node_name
        mi.description = self.description
        mi.type = self.monitoring_type
        # PC 필드가 있는 경우 설정
        try:
            mi.pc.Hostname = self.host_name
            mi.pc.ip = socket.gethostbyname(socket.gethostname())
        except:
            pass  # pc 필드가 없는 경우 무시
            
        self.ma.info.append(mi)

        self.is_initialised = False
        self.autoPublishing = autoPublishing

        # aggregation
        self.pub_times = 0
        self.aggregation_dict = {}
        self.aggregation_dict[self.host_name + self.node_name] = {}

    def init_ros(self):
        self.pub = rospy.Publisher('/monitoring', MonitoringArray, queue_size=20)

        if self.autoPublishing:
            try:
                frequency = 1
                frequency = rospy.get_param(rospy.get_name() + '/monitoring/frequency', 1)
                if frequency == 0:
                    rospy.logerr("frequency can not be 0, using 1")
                    frequency = 1
                duration = 1.0/frequency
                self.timer = rospy.Timer(rospy.Duration(duration), self.timercallback)
            except KeyError:
                rospy.logerr("monitoring frequency not set (%s/monitoring/frequency)", rospy.get_name())
                quit()

        self.is_initialised = True

    def timercallback(self, event):
        self.publish()

    def addValue(self, key, value, unit, target, errorlevel, monitor_mode=AggregationStrategies.LAST):
        def aggregation(mode):
            switcher = {
                1: AggregationStrategies.LAST,
                2: AggregationStrategies.FIRST,
                3: AggregationStrategies.MIN,
                4: AggregationStrategies.MAX,
                5: AggregationStrategies.AVG,
            }
            return switcher.get(mode, None)
            
        if not self.is_initialised:
            self.init_ros()
            
        # Check if key contains whitespace
        if " " in key:
            rospy.logwarn("[%s] whitespaces are not allowed in monitoring keys!", self.node_name)
            
        # 중요: 메시지 생성 및 값 설정
        kv = KeyValue()
        kv.key = str(key)  # 직접 설정
        kv.value = value  # 직접 설정
        kv.unit = unit  # 직접 설정
        kv.errorlevel = errorlevel  # 직접 설정
        kv.target = target
        #print(type(kv.value), type(kv.unit), type(kv.unit), type(kv.errorlevel), type(kv.target))
        # 디버깅: 값이 설정되었는지 직접 확인
        #rospy.loginfo(f"DEBUG: Created KeyValue with key='{kv.key}', value='{kv.value}', unit='{kv.unit}'")
        
        # 집계 전략에 따른 처리
        if (self.host_name + self.node_name) in self.aggregation_dict:
            if key in self.aggregation_dict[self.host_name + self.node_name]:
                # 여기서 집계 전략에 따라 값 처리
                if aggregation(monitor_mode) == AggregationStrategies.AVG:
                    if rospy.get_rostime() - self.aggregation_dict[self.host_name + self.node_name][key]['Duration'] < rospy.Duration(5):
                        self.aggregation_dict[self.host_name + self.node_name][key]['num'] += 1
                        self.aggregation_dict[self.host_name + self.node_name][key]['Sum'] += value
                        # 값을 누적만 하고 아직 메시지에 추가하지 않음
                        return
                    else:
                        # 평균 계산
                        avg_value = self.aggregation_dict[self.host_name + self.node_name][key]['Sum'] / \
                                   (self.aggregation_dict[self.host_name + self.node_name][key]['num'] + 0.001)
                        kv.value = str(avg_value)
                        self.aggregation_dict[self.host_name + self.node_name][key] = {
                            'num': 0, 'Value': 0, 'Sum': 0, 'Duration': rospy.get_rostime()
                        }
                elif aggregation(monitor_mode) == AggregationStrategies.FIRST and self.pub_times > 0:
                    # FIRST 전략이고 이미 한번 발행했으면 무시
                    return
                elif aggregation(monitor_mode) == AggregationStrategies.MAX:
                    if value > self.aggregation_dict[self.host_name + self.node_name][key]['Value']:
                        self.aggregation_dict[self.host_name + self.node_name][key]['Value'] = value
                    kv.value = str(self.aggregation_dict[self.host_name + self.node_name][key]['Value'])
                elif aggregation(monitor_mode) == AggregationStrategies.MIN:
                    if value < self.aggregation_dict[self.host_name + self.node_name][key]['Value']:
                        self.aggregation_dict[self.host_name + self.node_name][key]['Value'] = value
                    kv.value = str(self.aggregation_dict[self.host_name + self.node_name][key]['Value'])
                # LAST는 기본값이므로 그대로 유지
            else:
                # 키가 처음 등장하면 초기화
                self.aggregation_dict[self.host_name + self.node_name][key] = {
                    'num': 1 if aggregation(monitor_mode) == AggregationStrategies.AVG else 0, 
                    'Value': value, 
                    'Sum': value if aggregation(monitor_mode) == AggregationStrategies.AVG else 0, 
                    'Duration': rospy.get_rostime()
                }
                if aggregation(monitor_mode) == AggregationStrategies.AVG:
                    # AVG면 값을 누적만 하고 아직 메시지에 추가하지 않음
                    return
        
        # 디버깅: 메시지에 추가하기 전 최종 확인
        # rospy.loginfo(f"DEBUG: Adding to message: key='{kv.key}', value='{kv.value}', unit='{kv.unit}'")
        
        # 메시지에 값 추가

        self.ma.info[0].values.append(kv)
        
        # 디버깅: 메시지에 추가된 후 확인
        # rospy.loginfo(f"DEBUG: Current message has {len(self.ma.info[0].values)} values")

    def publish(self):
        # 디버깅: 발행 전 메시지 상태 확인
        # rospy.loginfo(f"DEBUG: Publishing message with {len(self.ma.info[0].values)} values")
        # for i, val in enumerate(self.ma.info[0].values):
        #     rospy.loginfo(f"DEBUG: Publish value {i}: key='{val.key}', value='{val.value}', unit='{val.unit}'")
            
        self.ma.header.stamp = rospy.Time.now()
        self.ma.info[0].header.stamp = rospy.Time.now()
        self.pub.publish(self.ma)
        
        # FIRST 전략을 위해 발행 횟수 증가
        self.pub_times += 1
        
        # 발행 후 메시지 리셋
        self.resetMsg()

    def resetMsg(self):
                
        self.ma = MonitoringArray()
        mi = MonitoringInfo()
        mi.name = self.host_name + self.node_name
        mi.description = self.description
        mi.type = self.monitoring_type
        
        # PC 필드가 있는 경우 설정
        try:
            mi.pc.Hostname = self.host_name
            mi.pc.ip = socket.gethostbyname(socket.gethostname())
        except:
            pass  # pc 필드가 없는 경우 무시
            
        self.ma.info.append(mi)
        