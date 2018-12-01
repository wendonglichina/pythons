#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import getpass  
#将工作目录切换文次文件的目录
os.chdir(sys.path[0])
sys.path.append("../..")
from  vdo_logging.vdologging import init_log, write_log

from diagnose.srv import *

import rospy
from std_msgs.msg import String, Bool, UInt8MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Temperature
from vdo_msgs.msg import Realsense_warning, SystemDiagnose
import threading
import psutil
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from check_usb_serial import check_usb_serial_name
import re
import rosnode
import rosgraph

#配置参数
TOPIC_TIME_OUT = 3.0   #unit:second
NODE_TIME_OUT = 3.0
DEFAULT_CHECK_CYCLE = 3.0
MAX_CPU_USAGE  = 80
MAX_NODE_RESTART_COUNT = 5

# power params
LOW_POWER_VALUE = 24.179

USB_LIST = {'1': ['voltage', '02'], '4.4': ['ultrasonic', '03'], '6': ['imu', '04'], 'js0': ['joystick', '05']}

ros_node_list = [
# sensors
"/ultrasonic_node",
"/realsense_detect",
"/imu_node",
"/laser_node",

# drivers
"/robot_driver",
"/joystick",
"/warning_light_color",
"/voltage_monitor",

# navigation node
"/map_server_for_amcl",
"/map_server_for_movebase",
"/move_base",
"/amcl",

# other software node
"/environment_perception",
"/mobile_base_nodelet_manager",
"/cmd_vel_mux",
"/listen_pose_for_windows",
]

def get_node_info(node_name):
    master = rosgraph.Master('/rosnode')
    node_name = rosgraph.names.script_resolve_name('rosnode', node_name)
        
    node_api = rosnode.get_api_uri(master, node_name)
    if not node_api:
        #write_log("cannot contact [%s]: unknown node"% node_name)
        return
    
    #write_log("\ncontacting node %s ..."%node_api)

    return rosnode.get_node_connection_info_description(node_api, master)

#对重要topic的事实数据进行检测
class TopicDataChecker():
    def __init__(self, topic_name, msg_type, func, data_parser = None, level = "error", category = "software"):
        rospy.on_shutdown(self.shutdown)
        self.sub_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.func = func
        self.run_flag = True
        self.topic_name = topic_name
        self.data_parser = data_parser
        self.level = level
        self.category = category
        self.last_data_error_time = rospy.Time.now()

        sub = rospy.Subscriber(topic_name, msg_type, self.topic_cb)

        self.thread =  threading.Thread(target = self.listen_time, args=())
        self.thread.start()

    def topic_cb(self, msg):
        self.sub_time = rospy.Time.now()
        if  self.data_parser != None:
            error_dicts = self.data_parser(msg)
            if len(error_dicts) > 0:
                delta_time = self.sub_time - self.last_data_error_time
                if delta_time.secs >= TOPIC_TIME_OUT: 
                    self.func(self.level, self.category, "topic " + self.topic_name + " data error")
                    for error in error_dicts:
                        self.func(self.level, error, error_dicts[error])
                    
                    self.last_data_error_time = self.sub_time

    def listen_time(self):
        while self.run_flag and not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            delta_time = self.current_time - self.sub_time
            if delta_time.secs >= TOPIC_TIME_OUT:
                self.func(self.level, self.category, "topic " + self.topic_name + " not found")

            rospy.sleep(TOPIC_TIME_OUT)

    def shutdown(self):
        self.run_flag = False

#对重要node进行检测
#这里没必要每个node一个线程
class NodeChecker():
    def __init__(self, node_name, func, level = "error", category = "software"):
        rospy.on_shutdown(self.shutdown)

        self.node_name = node_name
        self.func = func
        self.level = level
        if category == "realsense":
            category = "06: " + category
        self.category = category
        self.run_flag = True

        self.thread =  threading.Thread(target = self.listen_node, args = ())
        self.thread.start()

    def listen_node(self):
        last_pid = -1
        current_pid = -1
        while self.run_flag and not rospy.is_shutdown():
            rospy.sleep(NODE_TIME_OUT)
            buf = get_node_info(self.node_name)
            if not buf:
                self.func(self.level, self.category, "node: " + self.node_name + " not found")
                continue
                
            start_index = int(buf.find("Pid: ")) + len("Pid: ")
            end_index = int(buf.find("Connections:")) - 1
            if start_index * end_index >= 0:
                current_pid = int(buf[start_index: end_index])

            #write_log(self.node_name + " pid is " + str(current_pid))
            if current_pid == -1:
                self.func(self.level, self.category, "node " + self.node_name + " not found")
            else:
                if last_pid != -1 and current_pid != last_pid:
                    last_pid = current_pid
                    outstr = "node " + self.node_name + " restart over tolerance"
                    self.func(self.level, self.category, outstr)
                else:
                    last_pid = current_pid

    def shutdown(self):
        self.run_flag = False

#周期性的检测某类参数
class CycleChecker():
    def __init__(self, check_func, check_cycle, level = "error"):
        rospy.on_shutdown(self.shutdown)
        self.check_cycle = check_cycle        
        self.check_func = check_func
        self.level = level
        self.run_flag = True

        self.thread =  threading.Thread(target = self.run, args = ())
        self.thread.start()

    def run(self,):
        while self.run_flag and not rospy.is_shutdown():
            self.check_func(self.level)
            time.sleep(self.check_cycle)

    def shutdown(self):
        self.run_flag = False

#主类
class Diagnose():
    #错误处理，error级别，种类，错误信息
    def deal_with_error(self, level, category, error):
        #日志输出
        write_log(level + "," + category + "," + error)
        
        #topic输出
        system_status_msg = SystemDiagnose()
        system_status_msg.level = level
        system_status_msg.category = category
        system_status_msg.description = error
        self.publisher.publish(system_status_msg)

    def encoder_data_checker(self, msg):
        data = msg.data
        error_dicts = {}
        lalive = data[data.index("lalive:") + len("lalive:")]
        if lalive != '1':
            error_dicts['left_dirver'] = "00: left controller is offline"

        ralive = data[data.index("ralive:") + len("ralive:")]
        if ralive != '1':
            error_dicts['right_dirver'] = "01: right controller is offline"

        return error_dicts

    def voltage_data_checker(self, msg):
        data = msg.data
        value = float(data)

        error_dicts = {}
        if value <= self.low_power:
            error_dicts['low_power'] = "12: power is low"

        return error_dicts
        

    def __init__(self):
        rospy.init_node('diagnose', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.ips = {}
        self.ips['laser'] = [rospy.get_param('~laser', "0.0.0.0"), "07"]
        self.ips['camera'] = [rospy.get_param('~camera', "0.0.0.0"), "08"]
        self.ips['infrared'] = [rospy.get_param('~infrared', "0.0.0.0"), "09"]
        self.ips['ap'] = [rospy.get_param('~ap', "0.0.0.0"), "10"]
        self.ips['station'] = [rospy.get_param('~station', "0.0.0.0"), "11"]
        write_log(self.ips)

        self.publisher = rospy.Publisher('/diagnose/system_status', SystemDiagnose, queue_size=5)

        self.low_power = LOW_POWER_VALUE
        set_low_power_service = rospy.Service('set_low_power', SetLowPower, self.srv_set_low_power)
        get_low_power_service = rospy.Service('get_low_power', GetLowPower, self.srv_get_low_power)

        # add needed nodes 
        for node in ros_node_list:
            if node == "/realsense_detect":
                NodeChecker(node, self.deal_with_error, "error", "realsense")
            else:
                NodeChecker(node, self.deal_with_error)

        # add needed topics
        # sensors data
        TopicDataChecker("/mobile_base/sensors/ultrasonic", String, self.deal_with_error)
        TopicDataChecker("/mobile_base/sensors/realsense_detect", Realsense_warning, self.deal_with_error)
        TopicDataChecker("/mobile_base/sensors/temperature", Temperature, self.deal_with_error)
        TopicDataChecker("/scan", LaserScan, self.deal_with_error)
        
        # drivers data
        TopicDataChecker("/odom", Odometry, self.deal_with_error)
        TopicDataChecker("/mobile_base/power/charging", Bool, self.deal_with_error)
        TopicDataChecker("/mobile_base/light/state", String, self.deal_with_error)
        TopicDataChecker("/mobile_base/power/voltage", String, self.deal_with_error, self.voltage_data_checker)
        TopicDataChecker("/mobile_base/wheel/encoder", String, self.deal_with_error, self.encoder_data_checker)
        
        # node data
        TopicDataChecker("/vrobot/pose", Pose2D, self.deal_with_error)

        CycleChecker(self.check_cpu_usage, DEFAULT_CHECK_CYCLE, "warning")
        CycleChecker(self.check_usb_exist, DEFAULT_CHECK_CYCLE)
        CycleChecker(self.check_ip_exist, DEFAULT_CHECK_CYCLE)

        rospy.spin()

    def shutdown(self):
        write_log("Stopping the node...")

    def srv_set_low_power(self, req):
        write_log("call srv_set_low_power")   

        self.low_power = req.lowpower
        return SetLowPowerResponse(self.low_power)

    def srv_get_low_power(self, req):
        write_log("call srv_get_low_power")   

        return GetLowPowerResponse(self.low_power)

    def readCpuInfo(self):
        f = open('/proc/stat')
        lines = f.readlines();
        f.close()
               
        for line in lines:
            line = line.lstrip()
            counters = line.split()

            if len(counters) < 5:
                continue

            if counters[0].startswith('cpu'):
                break

        total = 0
        for i in xrange(1, len(counters)):
            total = total + long(counters[i])

        idle = long(counters[4])
     
        return {'total':total, 'idle':idle}

    def calcCpuUsage(self, counters1, counters2):
        idle = counters2['idle'] - counters1['idle']
        total = counters2['total'] - counters1['total']
    
    def check_cpu_usage(self, level = "error"):
        category = 'cpu'
        counters1 = self.readCpuInfo()
        time.sleep(0.1)
        counters2 = self.readCpuInfo()
        cpu_sum = self.calcCpuUsage(counters1, counters2)
        if cpu_sum >= MAX_CPU_USAGE:
            self.deal_with_error(level, category, "13: cpu: over tolerance")

    def check_ip_exist(self, level = "error"):
        keys = self.ips.keys()
        for key in keys:
            if self.ips[key] == "0.0.0.0":
                write_log("%s is empty" % key)
                continue

            ipstr = "ping " + self.ips[key][0] + " -c 4"
            p = os.popen(ipstr).read()
            reg_receive = '\d%'
            match_receive = re.search(reg_receive, p)
            match_receive1 = re.search('ttl', p)
            if match_receive:
                if match_receive.group() == '0%' and match_receive1:
                    pass
                else:
                    self.deal_with_error(level, key, "%s: ip [%s] not found"%(self.ips[key][1], self.ips[key][0]))
            else:
                self.deal_with_error(level, key, "%s: ip [%s] not found"%(self.ips[key][1], self.ips[key][0]))

    def check_usb_exist(self, level = "error"):
        #直接字符串search
        usblist = check_usb_serial_name()
        #write_log(usblist)
        if usblist != None:
            for usb in usblist:
                self.deal_with_error(level, USB_LIST[usb][0],"%s: usb [%s] not found" % (USB_LIST[usb][1], usb))
        else:
            self.deal_with_error(level, "software", "check_usb_serial is failed!")


if __name__ == '__main__':
    # 提取py文件名并当作log文件的名字
    log_name = os.path.basename(sys.argv[0].split(".")[0]) + '.log'
    init_log(log_name)
    Diagnose()
