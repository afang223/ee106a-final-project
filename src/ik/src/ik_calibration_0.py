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
    [-0.144904296875, 0.52504296875, -2.2662880859375, 1.24596484375, 2.2595224609375, 1.983951171875, 2.1579580078125],
    [0.020359375, 0.412833984375, -1.9827265625, 1.155279296875, 2.0413662109375, 1.772779296875, 2.28231640625],
    [0.04221484375, 0.4160234375, -2.026986328125, 1.13558984375, 2.0293974609375, 1.799595703125, 2.2825224609375],
    [0.0541708984375, 0.418912109375, -2.070353515625, 1.124951171875, 2.048595703125, 1.8575654296875, 2.4026455078125],
    [0.0102548828125, 0.5962490234375, -2.38252734375, 1.35157421875, 2.35996875, 2.022708984375, 2.368703125],
    [0.213982421875, 0.4743408203125, -2.06355859375, 1.2466865234375, 2.1326591796875, 1.7721591796875, 2.40347265625],
    [0.2618212890625, 0.4753759765625, -2.0824892578125, 1.2393505859375, 2.1237763671875, 1.752134765625, 2.4374150390625],
]

NEUTRAL_JOINT_STATE = [-0.228099609375, 0.3038212890625, -2.0829228515625, 1.1814384765625, 2.048802734375, 1.8156826171875, 2.1043388671875]
NEUTRAL_JOINT_STATE_2 = [0.2197578125, 0.1168359375, -1.927494140625, 1.1961015625, 1.7790029296875, 1.77174609375, 2.347916015625]
PICK_UP_JOINT_STATE = [-0.5379638671875, 0.975087890625, -2.3582724609375, 1.2941044921875, 2.744689453125, 1.5198671875, 2.01170214843755]

GRIPPER_SEQUENCE = [
    # 'open',
    # 'close',
    'close',
    # 'close',
    # 'open',
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
            # NEUTRAL_JOINT_STATE,
            # PICK_UP_JOINT_STATE,
            # NEUTRAL_JOINT_STATE,
            # NEUTRAL_JOINT_STATE_2,
            COLUMN_JOINT_STATES[column],
            # NEUTRAL_JOINT_STATE_2,
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