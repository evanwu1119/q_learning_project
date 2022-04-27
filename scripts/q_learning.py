#!/usr/bin/env python3

import rospy
import numpy as np
import os

# import custom message types from project
from q_learning.msg import QMatrix, QLearningReward, RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Initialize publishers and subscribers TODO: find type of msgs
        self.q_matrix_pub = rospy.Publisher("q_matrix", QMatrix, queue_size=10)
        self.robot_action_pub = rospy.Publisher("robot_action", RobotMoveObjectToTag, queue_size=10)
        rospy.Subscriber(self.reward, Reward, function())

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
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

        # Initialize Q-matrix [64 x 9], rows are states, columns are actions
        self.Q = np.array((64, 9))


    def get_reward(self):
        # gets the reward from the environment

    def q_learning_algorithm(self):
        t = 0 # time step
        converge = False # indicator for whether Q has converged
        state = 0 

        gamma = 0.8
        
        while converge != True:
            # choose random valid state uniformly
            possible_states = self.action_matrix[state,:] != -1

            # if we reached a final state (all three objects in front of tags)
            if possible_states == []: #check later
                state = 0
                action = reset_world # reference reset_world.py
            # if continuing trajectory
            else: 
                next_state = np.random.choice(possible_states, size = 1)
                action = self.action[state, next_state]
                publish action to robot_move # need a lock to make sure 
                while not message received: rospy.spin()
                reward = subscriber # check that we are actually getting a reward

                # Update Q_matrix 
                self.Q[state, action] = reward + (gamma * np.max(self.Q[next_state, :]))
                
                # Set next state
                state = next_state
            
            t += 1
    

    def run(self):
        self.q_learning_algorithm()


    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return

if __name__ == "__main__":
    node = QLearning()
    node.run()
    rospy.spin()
