#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import time

class DrawError(object):
    odom_time, cb_time = 0, 0
    x_odom = []
    y_odom = []
    x_rf2o = []
    y_rf2o = []
    def __init__(self):
        pass
    def isCBAlive(self):
        return False if self.cb_time - self.odom_time > 2 else True
    
    def odomCallBack(self, data):
        self.x_odom.append(data.pose.pose.position.x)
        self.y_odom.append(data.pose.pose.position.y)
        self.odom_time = time.time()
        print("x : {0}, y : {1}\n".format(data.pose.pose.position.x, data.pose.pose.position.y))

    def rf2oCallBack(self, data):
        self.x_rf2o.append(data.pose.pose.position.x)
        self.y_rf2o.append(data.pose.pose.position.y)

    # check if data is not received. If so, exit and plot the figure
    def cbMonitor(self, plt):
        self.cb_time = time.time()
        if not self.isCBAlive():
            rospy.logwarn("No data received!Exit!")
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node("drawingError")
    de = DrawError()

    # gazebo's odom, without nosie
    rospy.Subscriber("/odom", Odometry, de.odomCallBack)
    # rf2o_odometry's odom
    rospy.Subscriber("odom_rf2o", Odometry, de.rf2oCallBack)
    rospy.Timer(rospy.Duration(2), de.cbMonitor)
    plt.title("odom display")

    while not rospy.is_shutdown():
        if de.cbMonitor(plt) == True:
            break
    plt.plot(de.x_odom, de.y_odom, color="b", label="/odom")
    plt.plot(de.x_rf2o, de.y_rf2o, color="r", label="/odom_rf2o")
    plt.legend()
    plt.show()