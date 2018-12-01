#!/usr/bin/env python
# encoding: utf-8
import rosgraph
import rosnode
import re
import time
import psutil
import threading
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, UInt8MultiArray
import rospy
import sys
import os
import getpass

# 将工作目录切换至次文件的目录
os.chdir(sys.path[0])
sys.path.append("../modules/")
from loc_logging.loclogging import init_log, write_log

# 配置参数
TOPIC_TIME_OUT = 3.0  # unit:second
NODE_TIME_OUT = 3.0
DEFAULT_CHECK_CYCLE = 3.0
MAX_NODE_RESTART_COUNT = 5

lidar_node = "/velodyne_ppointclouds"
lidar_topic= "/velodyne_points"
ip_address = "192.168.1.201"

# 对重要topic的事实数据进行检测
class TopicDataChecker():
    def __init__(self, topic_name, msg_type, func, data_parser=None, level="error", category="software"):
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

        self.ips = {}
        self.ips['lidar'] = [ip_address]
        write_log(self.ips)

        sub = rospy.Subscriber(topic_name, msg_type, self.topic_cb)

        self.thread = threading.Thread(target=self.listen_time, args=())
        self.thread.start()

    def topic_cb(self, msg):
        self.sub_time = rospy.Time.now()
        if self.data_parser != None:
            error_dicts = self.data_parser(msg)
            if len(error_dicts) > 0:
                delta_time = self.sub_time - self.last_data_error_time
                if delta_time.secs >= TOPIC_TIME_OUT:
                    self.func(self.level, self.category, "topic " +
                              self.topic_name + " data error")
                    for error in error_dicts:
                        self.func(self.level, error, error_dicts[error])

                    self.last_data_error_time = self.sub_time

    def listen_time(self):
        while self.run_flag and not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            delta_time = self.current_time - self.sub_time
            if delta_time.secs >= TOPIC_TIME_OUT:
                self.func(self.level, self.category, "topic " +
                          self.topic_name + " not found")

	    self.check_ip_exist()
   
            rospy.sleep(TOPIC_TIME_OUT)
  
    def check_ip_exist(self, level="error"):
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
                    self.func(level, key, ": ip [%s] not found" % (self.ips[key][0]))
            else:
                self.func(level, key, ": ip [%s] not found" % (self.ips[key][0]))


    def shutdown(self):
        self.run_flag = False

# 主类
class Diagnose():
    # 错误处理
    def deal_with_error(self, level, category, error):
        # 日志输出
        write_log(level + "," + category + "," + error)

    def __init__(self):
        rospy.init_node('diagnose', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        TopicDataChecker(lidar_topic, PointCloud2, self.deal_with_error)

        rospy.spin()

    def shutdown(self):
        write_log("Stopping the node...")

    

if __name__ == '__main__':
    # 提取py文件名并当作log文件的名字
    log_name = os.path.basename(sys.argv[0].split(".")[0]) + '.log'
    init_log(log_name)
    Diagnose()
