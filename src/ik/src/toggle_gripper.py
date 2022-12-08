#!/usr/bin/env python
import roslib; roslib.load_manifest('ik')

import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from ik.msg import command

from path_planner import PathPlanner
from intera_interface import gripper as robot_gripper

try:
    from controller import Controller
except ImportError:
    pass

import actionlib

class IKCommand():
    def __init__(self):
        self.right_gripper = robot_gripper.Gripper('right_gripper')
        print('Calibrating...')
        self.right_gripper.calibrate()
        rospy.sleep(2.0)

        self.right_gripper.close()

    def calibrate(self, input):
        if input == 'c':
            self.right_gripper.close()
        else:
            self.right_gripper.open()

    def listener(self):
        rospy.Subscriber("command", command, self.callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    ik_command = IKCommand()
    while True:
        col = int(input("Enter c or o"))
        ik_command.calibrate(input)