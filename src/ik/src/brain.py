#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
import random
import math

# TODO: custom message
from std_msgs.msg import String
from ik.msg import board_state, board_row, command
from strategy import minimax

class Brain():
    def __init__(self):
        self.pub = rospy.Publisher('command', command, queue_size=10)

    def callback(self, message):
        # # Get board state
        # print(rospy.get_name() + ": ")
        # print(message)
        # print()

        # Send column command
        player = message.player
        if player == 'RED':
            pub_msg = command()
            # pub_msg.column = random.randint(0, 6)

            board_obj = message.board
            board = []
            for i in range(6):
                board.append(list(board_obj[i].board_row))
            board = np.array(board)
            board = np.flip(board, 0)
            print(board)
            col, minimax_score, dict_of_values = minimax(board, 6, -math.inf, math.inf, True)
            print(col, minimax_score, dict_of_values)
            pub_msg.column = col
            
            self.pub.publish(pub_msg)

    def listener(self):
        rospy.Subscriber("board_state", board_state, self.callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('brain', anonymous=True)
    brain = Brain()
    brain.listener()