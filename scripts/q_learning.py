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

        # Initialize publishers and subscribers
        self.q_matrix_pub = rospy.Publisher("q_matrix", QMatrix, queue_size=10)
        self.robot_action_pub = rospy.Publisher("robot_action", RobotMoveObjectToTag, queue_size=10)
        rospy.Subscriber("q_learning/reward", QLearningReward, self.get_reward())

        self.msgReceived = False

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

        # Variables for convergence
        self.convCount = 0 # count how many iterations since last significant change for convergence check
        self.convChange = 0.0001 # maximum allowed change for convergence
        self.convNum = 100 # number of iterations with no change before matrix considered converged


    def get_reward(self, data):
        # gets the reward from the environment
        self.reward = data.reward
        self.msgReceived = True

    def is_converged(self, prevQ):
        diffs = np.subtract(prevQ, self.Q)
        # absDiffs = np.absolute(diffs)
        maxDiff = max( max(diffs), abs(min(diffs)) ) # accounts for negative difference
        self.convCount = self.convCount + 1 if maxDiff < convChange else 0
        return self.convCount >= convNum


    def q_learning_algorithm(self):
        t = 0 # time step
        converged = False # indicator for whether Q has converged
        state = 0

        gamma = 0.8

        while converged == False:
            # get list of possible next states
            possible_states = [i for i in len(self.action_matrix[state]) if self.action_matrix[state][i] != -1]

            # if we reached a final state (all three objects in front of tags), the world resets itself, so we update our current state
            if (possible_states == []).all(): # using numpy.all() for array comparison
                state = 0
                possible_states = [i for i in len(self.action_matrix[state]) if self.action_matrix[state][i] != -1]

            # continue regardless of whether reset or not
            next_state = np.random.choice(possible_states, size = 1)
            action_num = self.action[state][next_state]

            # publish chosen action
            action = RobotMoveObjectToTag()
            colorID = self.actions[action][1]
            action.robot_object = "pink" if colorID==0 else "green" if colorID==1 else "blue"
            robot_object.tag_id = self.actions[action][1]
            self.action_pub.publish(action)

            while not self.msgReceived: rospy.spin()
            reward = self.reward # check that we are actually getting a reward
            self.msgReceived = False

            # save prev matrix for convergence check
            prevQ = self.Q.copy() # deep copy with numpy.ndarray.copy()
            # Update Q_matrix
            self.Q[state, action] = reward + (gamma * np.max(self.Q[next_state, :]))

            # Set next state
            state = next_state

            t += 1
            converged = is_converged(prevQ)


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
