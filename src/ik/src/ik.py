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

from path_planner import PathPlanner
from intera_interface import gripper as robot_gripper

try:
    from controller import Controller
except ImportError:
    pass

import actionlib

JOINT_STATES = [
    # [-0.393150390625, -0.65659765625, 0.004650390625, 0.811326171875, -0.1521083984375, 1.4538173828125, 1.36056640625],
    # [-0.4279208984375, -0.266490234375, 0.0094140625, 0.924732421875, -0.0955068359375, 1.0049033203125, 1.36056640625],
    # [-0.393150390625, -0.65659765625, 0.004650390625, 0.811326171875, -0.1521083984375, 1.4538173828125, 1.36056640625],
    # [-0.0889716796875, -0.80162109375, 0.1016875, 1.157419921875, -0.049091796875, 1.2567958984375, 1.809466796875],

    [-0.428326171875, -0.46364453125, 0.022119140625, 0.7239169921875, -0.0228828125, 1.3279873046875, 1.3593271484375],
    [-0.448734375, -0.2357978515625, 0.0252958984375, 0.8605849609375, -0.045966796875, 1.0474453125, 1.2405458984375],
    [-0.428326171875, -0.46364453125, 0.022119140625, 0.7239169921875, -0.0228828125, 1.3279873046875, 1.3593271484375],
    [-0.07706640625, -0.676259765625, -0.0448876953125, 1.235111328125, 0.1171748046875, 1.01977734375, 1.68794921875],
]

GRIPPER_STATE = [
    'open',
    'close',
    'close',
    'open',
]

def main():
    planner = PathPlanner("right_arm")

    # size = np.array([.8, .1, .8]) #np.array([0.4, 1.2, 0.1])
    # name = 'block'
    # pt = Point(0.82, -0.05, 0.06)
    # quat = Quaternion(0.0, 0.0, 0.0, 1.0)
    # pose = Pose(pt, quat)
    # header = Header()
    # pose = PoseStamped(header, pose)

    # planner.add_box_obstacle(size, name, pose)

    # size = np.array([.8, .8, .1]) #np.array([0.4, 1.2, 0.1])
    # name = 'block2'
    # pt = Point(0.52, 0.00, 0.90)
    # quat = Quaternion(0.0, 0.0, 0.0, 1.0)
    # pose = Pose(pt, quat)
    # header = Header()
    # pose = PoseStamped(header, pose)

    # planner.add_box_obstacle(size, name, pose)

    Kp = 2.0 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3]) #0.2
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

    right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    while not rospy.is_shutdown():
        for joint_state, gripper_state in zip(JOINT_STATES, GRIPPER_STATE):
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

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    main()