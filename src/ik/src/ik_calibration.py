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

COLUMN_JOINT_STATES = [
    [-0.30933984375, -0.708130859375, 0.107501953125, 1.2543017578125, -0.02377734375, 1.0012861328125, 1.48803515625],
    [-0.219853515625, -0.6972216796875, 0.0414873046875, 1.244673828125, 0.0063046875, 0.9859609375, 1.5469091796875],
    [-0.1443896484375, -0.6748974609375, 0.006015625, 1.1945224609375, 0.0050654296875, 1.024693359375, 1.636962890625],
    [-0.0843876953125, -0.6241630859375, -0.0361220703125, 1.12091796875, 0.0508212890625, 1.08245703125, 1.64680078125],
    [0.0038681640625, -0.56478515625, -0.106658203125, 1.028048828125, 0.096138671875, 1.153041015625, 1.6825126953125],
    [0.036857421875, -0.5051943359375, -0.0800029296875, 0.912404296875, 0.07162109375, 1.2579423828125, 1.752876953125],
    [0.1875859375, -0.466076171875, -0.2710927734375, 0.915716796875, 0.191705078125, 1.1441455078125, 1.7181982421875],
]

NEUTRAL_JOINT_STATE = [-0.4385087890625, -0.784578125, 0.0325087890625, 1.267376953125, 0.0058916015625, 0.99798046875, 1.3397265625]
PICK_UP_JOINT_STATE = [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375]

GRIPPER_SEQUENCE = [
    'open',
    'close',
    # 'close',
    # 'open',
    # 'open',
]

class IKCommand():
    def __init__(self):
        self.planner = PathPlanner("right_arm")

        Kp = 2.0 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3]) #0.2
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

        self.controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

        self.right_gripper = robot_gripper.Gripper('right_gripper')
        print('Calibrating...')
        self.right_gripper.calibrate()
        rospy.sleep(2.0)

    def calibrate(self, column):
        joint_state_sequence = [
            NEUTRAL_JOINT_STATE,
            # PICK_UP_JOINT_STATE,
            # NEUTRAL_JOINT_STATE,
            COLUMN_JOINT_STATES[column],
            # NEUTRAL_JOINT_STATE,
        ]

        if not rospy.is_shutdown():
            for joint_state, gripper_state in zip(joint_state_sequence, GRIPPER_SEQUENCE):
                try:
                    # Might have to edit this . . . 
                    plan = self.planner.plan_to_pose(joint_state)
                    # print("LENGTH OF PLAN: ", len(plan))
                    # input("Press <Enter> to move the right arm to goal pose 1: ")
                    if not self.controller.execute_plan(plan[1], log=False): 
                        raise Exception("Execution failed")
                except Exception as e:
                    print(e)
                    traceback.print_exc()
                
                rospy.sleep(0.5)
                if gripper_state == 'open':
                    self.right_gripper.open()
                elif gripper_state == 'close':
                    self.right_gripper.close()
                else:
                    raise ValueError
        else:
            raise ValueError('not on')

    def listener(self):
        rospy.Subscriber("command", command, self.callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    ik_command = IKCommand()
    while True:
        col = int(input("Enter column number"))
        ik_command.calibrate(col)