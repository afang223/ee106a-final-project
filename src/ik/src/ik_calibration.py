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
    [-0.0656201171875, 0.0815068359375, -1.3483896484375, 0.6785361328125, 1.3589326171875, 1.4072451171875, 2.5461123046875],
    [-0.017265625, 0.048447265625, -1.3030380859375, 0.608830078125, 1.202529296875, 1.3795380859375, 2.7165107421875],
    [0.0318017578125, 0.01590234375, -1.2059453125, 0.656798828125, 1.181509765625, 1.352244140625, 2.7165107421875],
    [0.0919130859375, -0.0121142578125, -1.1402822265625, 0.6969619140625, 1.152046875, 1.2972431640625, 2.7084677734375],
    [0.1031494140625, 0.0565830078125, -1.3181064453125, 0.6402314453125, 1.28597265625, 1.4012412109375, 2.784771484375],
    [0.1395400390625, 0.0479326171875, -1.2743525390625, 0.639076171875, 1.2736162109375, 1.3766455078125, 2.7955],
    [0.2213017578125, -0.0119091796875, -1.146662109375, 0.588662109375, 1.0272109375, 1.281931640625, 2.990765625],
]

NEUTRAL_JOINT_STATE = [-0.228099609375, 0.3038212890625, -2.0829228515625, 1.1814384765625, 2.048802734375, 1.8156826171875, 2.1043388671875]
NEUTRAL_JOINT_STATE_2 = [0.0689130859375, 0.0055986328125, -1.582330078125, 0.7375576171875, 1.54199609375, 1.5721171875, 2.66105859375]
PICK_UP_JOINT_STATE = [-0.4505625, 0.6529326171875, -1.898267578125, 0.9017568359375, 2.216955078125, 1.357421875, 2.050939453125]

GRIPPER_SEQUENCE = [
    'open',
    'close',
    'close',
    'close',
    'open',
    'open',
    'open',
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
            PICK_UP_JOINT_STATE,
            NEUTRAL_JOINT_STATE,
            NEUTRAL_JOINT_STATE_2,
            COLUMN_JOINT_STATES[column],
            NEUTRAL_JOINT_STATE_2,
            NEUTRAL_JOINT_STATE,
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