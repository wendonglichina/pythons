#!/usr/bin/env python
# -*- coding:utf-8 -*-
import sys
import os
import getpass  
#将工作目录切换文次文件的目录
os.chdir(sys.path[0])
sys.path.append("../..")
#from  vdo_logging.vdologging import init_log, write_log

import serial  
import time
import struct

import rospy
#from geometry_msgs.msg import Twist, Quaternion
#from std_msgs.msg import String, Bool, Int32
#from vdo_msgs.msg import Realsense_warning
#from vdo_msgs.msg import Environment_perception as e_msg
#from sensor_msgs.msg import LaserScan
from math import atan2, fabs, sqrt, sin
import math

import threading
import psutil

#from get_usb_serial import get_usb_serial_name

class serials():
    def __init__(self):
        #usb_id = rospy.get_param('~USB_port', 0)
        #print "usbid is:%d"%(usb_id)

        #获取串口名称
        #serial_name = get_usb_serial_name(usb_id)
        serial_name = "/dev/ttyUSB0"
        if serial_name == None:
            print "could not get serial name"
            return

        print "serial_name is:%s"%(serial_name)

        self.start_serial(serial_name)
    def start_serial(self, s_name):
        t = serial.Serial(s_name, 460800, timeout=3)     
        print t
if __name__ == '__main__':
    # 提取py文件名并当作log文件的名
    
    serials()
