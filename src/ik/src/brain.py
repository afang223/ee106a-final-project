#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
import random

# TODO: custom message
from std_msgs.msg import String
from ik.msg import board_state, board_row, command

class Brain():
    def __init__(self):
        self.pub = rospy.Publisher('command', command, queue_size=10)

    def callback(self, message):
        # Get board state
        print(rospy.get_name() + ": ")
        print(message)
        print()

        # Send column command
        
        pub_msg = command()
        pub_msg.column = random.randint(0, 6)
        self.pub.publish(pub_msg)

    def listener(self):
        rospy.Subscriber("board_state", board_state, self.callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('brain', anonymous=True)
    brain = Brain()
    brain.listener()