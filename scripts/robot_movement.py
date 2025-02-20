#!/usr/bin/env python3

import rospy, cv2, cv_bridge, moveit_commander, math
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

        # set up arm groups
        self.move_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # declare arm commands
        self.gripper_open = [0.019, 0.019]
        self.gripper_close = [-0.005, -0.005]
        self.arm_up = [
            0.0,
            math.radians(-50.0),
            math.radians(0.0),
            math.radians(-20.0)           
        ]
        self.arm_down = [
            0.0,
            math.radians(35.0),
            math.radians(10.0),
            math.radians(-50.0)
        ]

        # queue to store actions to perform
        self.action_queue = []

        # set up necessary variables
        self.min_dist = 0 # distance to closest object
        self.min_front = 0 # distance to closest object in front 10 degrees
        self.max_dist = 0.735 # max distance from objects as origin position
        self.curr_image = np.array([]) # current camera image


    def execute_actions(self):
        # loop to perform robot actions
        finished_actions = 3
        self.max_dist = self.min_dist

        self.move_arm.go(self.arm_up, wait=True)
        rospy.sleep(5)
        self.move_arm.stop()

        while finished_actions and not rospy.is_shutdown():
            # wait if no actions to perform
            while len(self.action_queue) == 0:
                print("waiting...")
                rospy.sleep(0.1)

            action = self.action_queue[0]

            # call necessary functions to perform the action
            print("moving to object " + action.robot_object)
            self.move_object(object = action.robot_object)
            print("moving to origin...")
            self.move_to_origin()
            print("moving to tag..." + str(action.tag_id))
            self.move_to_tag(tag = action.tag_id)
            print("moving to origin...")
            self.move_to_origin()

            # pop the action
            self.action_queue.pop(0)
            finished_actions -= 1


    def robot_action_received(self, data):
        # append robot actions to queue
        self.action_queue.append(data)


    def scan_received(self, data):
        # get distance from closest object that's not zero 
        rs = np.nan_to_num(data.ranges)
        self.min_dist = np.amin(rs[np.nonzero(rs)])

        # get min distance in front that's not zero
        front = np.concatenate((rs[350:359], rs[0:10]))
        deg_10 = front[np.where(front > 0.1)]
        
        if deg_10.size > 0:
            self.min_front = np.amin(deg_10)
        else:
            self.min_front = np.inf


    def image_received(self, data):
        # get current image from robot camera 
        self.curr_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')


    def move_object(self, object):
        # find and move to object of specified color
        while not rospy.is_shutdown():
            hsv = cv2.cvtColor(self.curr_image, cv2.COLOR_BGR2HSV)

            # HSV = [0-179, 0-255, 0-255]
            if object == 'pink':
                lower = np.array([149, 128, 95]) # H = 300, S = 1/2, V = 3/8
                upper = np.array([164, 255, 255]) # H = 330, S = 1, V = 1
            elif object == 'green':
                lower = np.array([34, 128, 95]) # H = 70, S = 1/2, V = 3/8
                upper = np.array([59, 255, 255]) # H = 120, S = 1, V = 1
            else: # blue
                lower = np.array([84, 128, 95]) # H = 170, S = 1/2, V = 3/8
                upper = np.array([104, 255, 255]) # H = 230, S = 1, V = 1

            mask = cv2.inRange(hsv, lower, upper)
            w = self.curr_image.shape[1]
            M = cv2.moments(mask)

            # scan surroundings if no color in image
            if M['m00'] == 0:
                twist = Twist()
                twist.angular.z = 0.05
                self.robot_movement_pub.publish(twist)

            # if object found
            else:
                # center of colored pixels
                cx = int(M['m10']/M['m00'])

                # keep rotating until object is centered
                # if centered, move towards it
                twist = Twist()
                err = w/2 - cx

                if np.abs(err) < 30:
                    twist.linear.x = 0.07
                    twist.angular.z = 0.001 * err
                else:
                    # control rotation within 0.05
                    if np.abs(0.001 * err) > 0.05:
                        twist.angular.z = err/np.abs(err) * 0.05
                    else:
                        twist.angular.z = 0.001 * err

                self.robot_movement_pub.publish(twist)

                stop_dist = 0.237

                # pick up tube if within range
                if self.min_front - stop_dist < 0.01:
                    self.robot_movement_pub.publish(Twist())
                    
                    # pick_up_tube
                    print("GRIPPER OPEN")
                    self.move_gripper.go(self.gripper_open, wait=True)
                    rospy.sleep(2)
                    print("ARM DOWN")
                    self.move_arm.go(self.arm_down, wait=True)
                    rospy.sleep(5)
                    print("GRIPPER CLOSE")
                    self.move_gripper.go(self.gripper_close, wait=True)
                    rospy.sleep(2)
                    print("ARM UP")
                    self.move_arm.go(self.arm_up, wait=True)
                    rospy.sleep(5)
                    self.move_arm.stop()
                    
                    return

            cv2.waitKey(3)


    def move_to_origin(self):
        # move back to origin, basically just moving in reverse direction
        time = 6.0
        rate = 10
        r = rospy.Rate(rate)
        while not rospy.is_shutdown() and time >= 0:
            twist = Twist()
            twist.angular.z = 0.5
            self.robot_movement_pub.publish(twist)
            r.sleep()
            time -= 1/rate

        time = 2.0
        while not rospy.is_shutdown() and time >= 0:
            twist = Twist()
            twist.linear.x = 0.2
            self.robot_movement_pub.publish(twist)
            r.sleep()
            time -= 1/rate
        
        # stop at origin
        self.robot_movement_pub.publish(Twist())

    
    def move_to_tag(self, tag): 
        # find and move to specified tag
        while not rospy.is_shutdown():
            grayscale = cv2.cvtColor(self.curr_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(grayscale, self.aruco_dict)

            w = self.curr_image.shape[1]

            # scan surroundings if desired tag not in image
            if len(corners) == 0 or not [tag] in ids:
                twist = Twist()
                twist.angular.z = 0.05
                self.robot_movement_pub.publish(twist)

            # if tag found
            else:
                # get tag corners and find center 
                for i, t in enumerate(ids):
                    if t[0] == tag:
                        tag_ind = i

                topLeft = corners[tag_ind][0][0]
                bottomRight = corners[tag_ind][0][2]
                cx = int((topLeft[0] + bottomRight[0]) / 2.0)

                # keep rotating until tag is centered
                # if centered, move towards it
                twist = Twist()
                err = w/2 - cx

                if np.abs(err) < 30:
                    twist.linear.x = 0.07
                    twist.angular.z = 0.001 * err
                else:
                    # control rotation within 0.05
                    if np.abs(0.001 * err) > 0.05:
                        twist.angular.z = err/np.abs(err) * 0.05
                    else:
                        twist.angular.z = 0.001 * err

                self.robot_movement_pub.publish(twist)

                stop_dist = 0.4

                # put down tube if within range
                if self.min_front - stop_dist < 0.01:
                    self.robot_movement_pub.publish(Twist())
                    
                    # put_down_tube
                    print("ARM DOWN")
                    self.move_arm.go(self.arm_down, wait=True)
                    rospy.sleep(5)
                    print("GRIPPER OPEN")
                    self.move_gripper.go(self.gripper_open, wait=True)
                    rospy.sleep(2)
                    print("ARM UP")
                    self.move_arm.go(self.arm_up, wait=True)
                    rospy.sleep(5)
                    print("GRIPPER CLOSE")
                    self.move_gripper.go(self.gripper_close, wait=True)
                    rospy.sleep(2)
                    self.move_arm.stop()
                    return

            cv2.waitKey(3)



if __name__ == "__main__":
    sub = RobotMovement()
    rospy.sleep(3)
    sub.execute_actions()
    rospy.spin()