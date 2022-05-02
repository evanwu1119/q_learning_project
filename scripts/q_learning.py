#!/usr/bin/env python3

import rospy
import numpy as np
import os

# import custom message types from project
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        self.initialized = False 

        # Initialize this node
        rospy.init_node("q_learning")

        # Initialize publishers and subscribers
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        # Initialize variables to store information from reward subscriber
        self.reward = 0
        self.msg_received = False

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

        # Initialize Q-matrix [64 x 9], rows are states, columns are actions
        self.Q = np.zeros((64, 9))

        self.num_conv = 0 # track number of times matrix is converged

        self.initialized = True


    def get_reward(self, data):
        # called by reward subscriber to get reward after robot performs an action
        self.reward = data.reward
        self.msg_received = True


    def is_converged(self, prevQ, epsilon, conv):
        # checks if all elements in the Q matrix changed significantly (> epsilon)
        # if converged for conv iterations returns true
        abs_diff = np.abs(self.Q - prevQ)
        if np.amax(abs_diff) < epsilon:
            self.num_conv += 1
        
        return self.num_conv == conv


    def q_learning_algorithm(self):
        # main loop for the Q-learning algorithm
        # iteratively update the Q matrix by performing random actions until converged
        t = 0 
        state = 0 # index of current state
        gamma = 0.8 # discount factor

        converged = False # indicator for whether Q has converged
        epsilon = 0.001 # threshold for convergence
        conv = 100 # number of iterations repeated for convergence

        while not converged:
            # get list of possible next states
            possible_states = [i for i in range(self.action_matrix.shape[1]) if self.action_matrix[state,i] != -1]

            # if we reached a final state the world resets
            if len(possible_states) == 0:
                state = 0
                possible_states = [i for i in range(self.action_matrix.shape[1]) if self.action_matrix[state,i] != -1]

            # sample next state uniformly from possible transitions and get action required
            next_state = np.random.choice(possible_states)
            action_ind = int(self.action_matrix[state, next_state])
            action = self.actions[action_ind]

            # publish chosen action
            robot_action = RobotMoveObjectToTag()
            robot_action.robot_object = action['object']
            robot_action.tag_id = action['tag']
            self.robot_action_pub.publish(robot_action)
            
            # get reward, wait for subscriber to update
            # r = rospy.Rate(10)
            # while not self.msg_received and not rospy.is_shutdown(): 
            #     print("waiting...")
            #     r.sleep()
            reward = self.reward # check that we are actually getting a reward
            self.msg_received = False

            # save prev matrix for convergence check
            prevQ = np.copy(self.Q)

            # update Q_matrix and publish to topic as flattened row-major array
            self.Q[state, action_ind] = reward + (gamma * np.max(self.Q[next_state,:]))
            self.q_matrix_pub.publish(q_matrix = self.Q.flatten())

            # set next state as current state
            state = next_state

            # check for convergence
            converged = self.is_converged(prevQ, epsilon, conv)

            t += 1
        
        print("converged timestep: " + str(t))


    def save_q_matrix(self):
        # save the q_matrix to .csv file
        np.savetxt(path_prefix + "trained_q_matrix.csv", self.Q)


    def run(self):
        self.q_learning_algorithm()
        #self.save_q_matrix()
        

if __name__ == "__main__":
    node = QLearning()
    node.run()
    rospy.spin()
