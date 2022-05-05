#!/usr/bin/env python3

import rospy, os, cv2, cv_bridge
import numpy as np

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Vector3, Twist

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

        while np.where(self.Q[state,:] == 0).shape[0] != 0:
            # choose action with highest Q-value, or randomly if multiple are the max
            possible_states = self.Q[state,:]
            optimal_action_ind = np.random.choice(np.argwhere(possible_states == np.amax(possible_states)))

            # get action and publish 
            action = self.action_matrix[optimal_action_ind]
            self.robot_action_pub.publish(RobotMoveObjectToTag(action['object'], action['tag']))

            # update current state
            state = np.argwhere(self.action_matrix[state,:] == optimal_action_ind)



# subscriber node to execute published actions
class RobotMovement(object):
    def __init__(self):
        # initialize node
        rospy.init_node("robot_movement")

        # set up subscribers to robot actions and sensors      
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.robot_action_received)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)

        # set up publisher to move robot
        self.robot_movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # set up camera settings 
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_received)

        # get aruco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # queue to store actions to perform
        self.action_queue = []
        self.finished_actions = 0 # number of actions performed

        # set up necessary variables
        self.min_dist = 0 
        self.curr_image = []# something camera


    def execute_actions(self):
        # constant loop to perform robot actions
        while True:
            # wait if no actions to perform
            while len(self.action_queue) == 0:
                if self.finished_actions < 3:
                    rospy.sleep(1)
                else:
                    return

            # call necessary functions to perform the action
            self.move_object()
            self.move_to_origin()
            self.move_to_tag()
            self.move_to_origin()

            # pop the action
            self.action_queue.pop(0)
            self.finished_actions += 1


    def robot_action_received(self, data):
        # append robot actions to queue
        self.action_queue.append(data)


    def scan_received(self, data):
        # get distance from closest object (I don't think we need angles)
        ranges = np.nan_to_num(data.ranges)
        self.min_dist = np.amin(ranges)


    def image_received(self, data):
        # get current image from robot camera 
        self.


    def move_object(self, color):
        # detect tube, spin around if not in camera frame
        while not detecting_color: 
            # spin around
            publish.(angular velocity = 0.1)

        # once the tube is in the frame orient it to the center
        # reuse line follower code, stop dist = 0.237

        # move to correct distance 

        # pick up the tube, first open gripper to max, then descend and pick it up 


    def move_to_origin(self):
    # move back to origin 

    
    def move_to_tag(self, tag): 
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
        # move to the tag
        while not detecting_tag:
            #spin around

        # orient tag to center 

        # move to correct distance

        # drop tube



if __name__ == "main":
    pub = ChooseRobotActions()
    sub = RobotMovement()
    rospy.sleep(3)
    pub.choose_robot_actions()
    sub.execute_actions()
    rospy.spin()

