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

try:
    from controller import Controller
except ImportError:
    pass

import actionlib

POSITIONS = [
    [0.650, -0.106, -0.103], # Pick
    [0.635, 0.218, 0.172], # Place
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

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    while not rospy.is_shutdown():
        for x, y, z in POSITIONS:
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal_1, [orien_const])
                input("Press <Enter> to move the right arm to goal pose 1: ")
                if not controller.execute_plan(plan[1], log=False): 
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()

if __name__ == "__main__":
    rospy.init_node('moveit_node')
    main()