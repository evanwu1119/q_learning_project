#!/usr/bin/env python3

import rospy
import numpy as np
import os

from sensor_msgs.msg import LaserScan

# import custom message types from project
from q_learning_project.msg import RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"


class RobotActions(object):
    def __init__(self):
        self.initialized = False

        # initialize node
        rospy.init_node("robot_actions")

        # set up robot action publisher and subscriber
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)       
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.robot_action_received)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0,12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # read in converged Q-matrix
        self.Q = np.loadtxt(path_prefix + "trained_q_matrix.csv", delimiter = " ")

        self.initialized = True 

    
    def robot_get_actions(self):
        # get optimal actions from converged Q-matrix and publish
        state = 0 

        for i in range(3):
            # choose action with highest Q-value, or randomly if multiple are the max
            possible_states = self.Q[state,:]
            optimal_action_ind = np.random.choice(np.argwhere(possible_states == np.amax(possible_states)))

            # get action and publish (IDK if best approach, as we want actions to happen sequentially)
            action = self.action_matrix[optimal_action_ind]
            self.robot_action_pub.publish(RobotMoveObjectToTag(action['object'], action['tag']))
            # maybe wait for some sort of confirmation of action finished from subscriber

            # update current state
            state = np.argwhere(self.action_matrix[state,:] == optimal_action_ind)


    def robot_action_received(self, data):
        # subscriber function 
        actions = True 


    def scan_received(self, data):
        # get measurements from closest object so that we can move to each thing 


    def move_to_tube(self):
        # move to the tube and pick it up

    
    def move_to_tag(self): 
        # move to the tag