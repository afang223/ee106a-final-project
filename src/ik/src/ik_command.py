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
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
]

NEUTRAL_JOINT_STATE = [-0.428326171875, -0.46364453125, 0.022119140625, 0.7239169921875, -0.0228828125, 1.3279873046875, 1.3593271484375]
PICK_UP_JOINT_STATE = [-0.07706640625, -0.676259765625, -0.0448876953125, 1.235111328125, 0.1171748046875, 1.01977734375, 1.68794921875]

GRIPPER_SEQUENCE = [
    'open',
    'close',
    'close',
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

    def callback(self, message):
        column = message.column
        
        joint_state_sequence = [
            NEUTRAL_JOINT_STATE,
            PICK_UP_JOINT_STATE,
            NEUTRAL_JOINT_STATE,
            COLUMN_JOINT_STATES[column],
            NEUTRAL_JOINT_STATE,
        ]

        if not rospy.is_shutdown():
            for joint_state, gripper_state in zip(joint_state_sequence, GRIPPER_SEQUENCE):
                try:
                    # Might have to edit this . . . 
                    plan = planner.plan_to_pose(joint_state)
                    # print("LENGTH OF PLAN: ", len(plan))
                    # input("Press <Enter> to move the right arm to goal pose 1: ")
                    if not controller.execute_plan(plan[1], log=False): 
                        raise Exception("Execution failed")
                except Exception as e:
                    print(e)
                    traceback.print_exc()
                
                rospy.sleep(0.5)
                if gripper_state == 'open':
                    right_gripper.open()
                elif gripper_state == 'close':
                    right_gripper.close()
                else:
                    raise ValueError
        else:
            raise ValueError, 'not on'

    def listener(self):
        rospy.Subscriber("command", command, self.callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    ik_command = IKCommand()
    ik_command.listener()