#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

def follow_wall(scan):
    ####### STUDENT CODE START #######
    # use the scan data to appropriately modify 'speed' and 'steering_angle'

    speed = 1
    steering_angle = 1

    ####### STUDENT CODE END ####### 

    return (speed, steering_angle)






########################### Ignore Code Below ###########################
class WallFollower:
    # import ROS parameters from the "params.yaml" file.
    # access these variables in class functions with self:
    #   i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # setup laser scan subscriber
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,
                                         LaserScan,
                                         callback=self.scan_callback)

        # setup drive publisher
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC,
                                         AckermannDriveStamped,
                                         queue_size=1)

    def scan_callback(self, scan_msg):
        """Lidar callback function"""

        # get list of range measurements
        scan_data = scan_msg.ranges

        # call student's code for speed and angle, given scan
        drive_command = follow_wall(scan_data)

        print(drive_command)

        # create, populate and publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = drive_command[0]
        drive_msg.drive.steering_angle = drive_command[1]
        self.pub_drive.publish(drive_msg)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()