#!/usr/bin/env python3

import rospy, os
import numpy as np

# import custom message types from project
from q_learning_project.msg import RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# publisher node to choose and publish optimal actions
class ChooseRobotActions(object):
    def __init__(self):
        # initialize node
        rospy.init_node("choose_robot_actions")

        # set up robot action publisher
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # Fetch pre-built action matrix
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # read in converged Q-matrix
        self.Q = np.loadtxt(path_prefix + "trained_q_matrix.csv", delimiter = " ")


    def choose_robot_actions(self):
        # get optimal actions from converged Q-matrix and publish
        state = 0 
        num_actions = 3

        while num_actions:
            # choose action with highest Q-value, or randomly if multiple are the max
            possible_actions = self.Q[state,:]
            possible_inds = np.argwhere(possible_actions == np.amax(possible_actions)).flatten()
            optimal_action_ind = np.random.choice(possible_inds)

            # get action and publish 
            action = self.actions[int(optimal_action_ind)]
            self.robot_action_pub.publish(RobotMoveObjectToTag(robot_object = action['object'], tag_id = action['tag']))

            # update current state
            state = int(np.argwhere(self.action_matrix[state,:] == optimal_action_ind))
            num_actions -= 1



if __name__ == "__main__":
    pub = ChooseRobotActions()
    rospy.sleep(3)
    pub.choose_robot_actions()
    rospy.spin()