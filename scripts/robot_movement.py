#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Vector3, Twist

# import custom message types from project
from q_learning_project.msg import RobotMoveObjectToTag


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
        self.min_dist = 0 # distance to closest object
        self.max_dist = 1.0 # TODO: max distance from objects as origin position
        self.curr_image = [] # current camera image


    def execute_actions(self):
        # loop to perform robot actions
        while self.finished_actions < 3:
            # wait if no actions to perform
            while len(self.action_queue) == 0:
                rospy.sleep(0.1)

            action = self.action_queue[0]

            # call necessary functions to perform the action
            self.move_object(object = action['object'])
            self.move_to_origin()
            self.move_to_tag(tag = action['tag'])
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
        self.curr_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')


    def move_object(self, object):
        # find and move to object of specified color
        while True:
            hsv = cv2.cvtColor(self.curr_image, cv2.COLOR_BGR2HSV)

            # HSV = [0-179, 0-255, 0-255]
            if object == 'pink':
                lower = np.array([149, 63, 95]) # H = 300, S = 1/4, V = 3/8
                upper = np.array([164, 255, 255]) # H = 330, S = 1, V = 1
            elif object == 'green':
                lower = np.array([44, 63, 95]) # H = 90, S = 1/4, V = 3/8
                upper = np.array([59, 255, 255]) # H = 120, S = 1, V = 1
            else: # blue
                lower = np.array([104, 63, 95]) # H = 210, S = 1/4, V = 3/8
                upper = np.array([119, 255, 255]) # H = 240, S = 1, V = 1

            mask = cv2.inRange(hsv, lower, upper)
            # maybe slice image to get center?
            M = cv2.moments(mask)

            # scan surroundings if no color in image
            if M['m00'] == 0:
                twist = Twist()
                twist.angular.z = 0.1
                self.robot_movement_pub.publish(twist)

            # if object found
            else:
                # center of colored pixels
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # display moment as red circle
                cv2.circle(self.curr_image, (cx,cy), 10, (0,0,255), -1)

                # proportional control to turn and move towards object
                stop_dist = 0.237 # dist to pick up tube = 0.237
                ang_error = 0.005 * (self.curr_image.shape[1]/2 - cx)
                lin_error = 0.1 * (self.min_dist - stop_dist) 

                twist = Twist()
                twist.linear.x = lin_error
                twist.angular.z = ang_error 
                self.robot_movement_pub.publish(twist)

                # TODO: pick up tube if within range
                if (self.min_dist - stop_dist) < 0.01:
                    # pick_up_tube
                    return

            cv2.imshow("window", self.curr_image)
            cv2.waitKey(3)
            rospy.sleep(0.1)


    def move_to_origin(self):
        # move back to origin, basically just proportional control in reverse direction
        # this assumes minimal turning in the initial movement, so slicing images might help as long as we don't cut anything off
        while self.min_dist - self.max_dist < 0.01:
            lin_error = 0.1 * (self.min_dist - self.max_dist)
            twist = Twist()
            twist.linear.x = lin_error
            self.robot_movement_pub.publish(twist)

    
    def move_to_tag(self, tag): 
        # find and move to specified tag
        while True:
            grayscale = cv2.cvtColor(self.curr_image, cv2.COLOR_BGR2GRAY)
            # maybe slice image? 
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale, self.aruco_dict)

            # scan surroundings if desired tag not in image
            # might have to check if tag is in rejected points? 
            if len(corners) == 0 or not tag in ids:
                twist = Twist()
                twist.angular.z = 0.1
                self.robot_movement_pub.publish(twist)

            # if tag found
            else:
                # get tag corners and find center 
                tag_id = np.argwhere(ids = tag)
                (topLeft, _, bottomRight, _) = corners[tag_id,:,:,:].reshape((4,2))
                cx = int((topLeft[0] + bottomRight[0]) / 2.0)
                cy = int((topLeft[1] + bottomRight[1]) / 2.0)

                # draw center as red circle
                cv2.circle(self.curr_image, (cx,cy), 10, (0,0,255), -1)

                # proportional control to turn robot towards tag
                stop_dist = 0.3 # dist to put down tube
                ang_error = 0.005 * (self.curr_image.shape[1]/2 - cx)
                lin_error = 0.1 * (self.min_dist - 0.3) # dist to pick up tube = 0.237

                twist = Twist()
                twist.linear.x = lin_error
                twist.angular.z = ang_error 
                self.robot_movement_pub.publish(twist)

                # TODO: put down tube if within range
                if (self.min_dist - stop_dist) < 0.01:
                    # put_down_tube
                    return

            cv2.imshow("window", self.curr_image)
            cv2.waitKey(3)
            rospy.sleep(0.1)



if __name__ == "__main__":
    sub = RobotMovement()
    rospy.sleep(3)
    sub.execute_actions()
    rospy.spin()