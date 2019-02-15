#! /usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tft
import numpy as np
import rospy
import math
import copy

REAL_ROBOT = True

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()s
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self._pub, self._odom_sub = None, None
        if REAL_ROBOT:
            self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self._odom_sub = rospy.Subscriber('/odom', Odometry, callback=self._odom_callback)
        else:
            self._pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
            self._odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, callback=self._odom_callback)

        self._latest_odom = None

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _dist(self, p1, p2):
        dp = [p1.x - p2.x, p1.y - p2.y, p1.z - p2.z]
        return np.linalg.norm(np.array(dp))

    def _dist_rad(self, r1, r2):
        # returns abs value difference between r1 and r2
        return abs(self._normalize_rad(r1 - r2))

    def _normalize_rad(self, r1):
        # normalizes r1 to [-math.pi, math.pi]
        return ((r1 + math.pi) % (2 * math.pi)) - math.pi

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            line ar_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate counter-clockwise.
        """
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed 
        self._pub.publish(msg)
        # TODO: Fill out msg
        # TODO: Publish msg
        # rospy.logerr('Not implemented.')

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        if distance < 0.0:
            distance = -distance
            speed = -speed

        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self._latest_odom == None:
            rospy.sleep(1)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._latest_odom.pose.pose.position)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        delta = self._dist(self._latest_odom.pose.pose.position, start)
        lin_speed = speed
        while delta < distance:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            # direction = -1 if distance < 0 else 1
            # self.move(direction * speed, 0)
            self.move(lin_speed, 0)
            delta = self._dist(self._latest_odom.pose.pose.position, start)
            lin_speed = max(0.07, min(speed, distance-delta))
            rate.sleep()
        self.stop()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self._latest_odom == None:
            rospy.sleep(1)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._latest_odom.pose.pose.orientation)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        start_rads = self.quaternion_to_yaw(start)
        angular_distance = self._normalize_rad(angular_distance)  # puts into +-pi range
        direction = 1 if angular_distance < 0 else -1
        angular_distance = abs(angular_distance)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        delta = self._dist_rad(start_rads, \
                self.quaternion_to_yaw(self._latest_odom.pose.pose.orientation))
        turn_speed = speed
        # rospy.loginfo("delta: " + str(delta))
        while delta < angular_distance:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            self.move(0, direction * turn_speed)  # positive is clockwise
            delta = self._dist_rad(start_rads, \
                    self.quaternion_to_yaw(self._latest_odom.pose.pose.orientation))
            turn_speed = max(0.3, min(speed, angular_distance-delta))
            rate.sleep()
        self.stop()

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.move(0,0)
        # rospy.logerr('Not implemented.')

    def quaternion_to_yaw(self, q):
        # converts to rad
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0]
        y = m[1, 0]
        theta_rads = math.atan2(y, x)
        # theta_degs = theta_rads * 180 / math.pi
        return theta_rads
