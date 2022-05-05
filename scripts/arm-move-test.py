#!/usr/bin/env python3

import rospy
import moveit_commander
import math
from sensor_msgs.msg import LaserScan

class Robot(object):
    def __init__(self):
        rospy.init_node("tb3")

        # self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        # self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.Subscriber("/scan", LaserScan, self.process_scan)

    def process_scan(self, data):
        print(data.ranges[0])

    def run(self):
        gripper_open = [0.008, 0.008]
        self.move_group_gripper.go(gripper_open, wait=True)
        rospy.sleep(2)

        arm_up = [
            0.0,
            0.0,
            math.radians(10.0),
            0.0           
        ]
        self.move_group_arm.go(arm_up, wait=True)
        rospy.sleep(2)

        arm_down = [
            0.0,
            math.radians(35.0),
            math.radians(10.0),
            math.radians(-50.0)
        ]
        self.move_group_arm.go(arm_down, wait=True)
        rospy.sleep(2)

        gripper_close = [-0.005, -0.005]
        self.move_group_gripper.go(gripper_close, wait=True)
        rospy.sleep(2)

        self.move_group_arm.go(arm_up, wait=True)
        rospy.sleep(2)
        self.move_group_arm.stop()

if __name__ == '__main__':
    node = Robot()
    rospy.spin()
    # node.run()