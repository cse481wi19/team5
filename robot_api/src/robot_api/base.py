#! /usr/bin/env python

<<<<<<< HEAD
# TODO: import ????????_msgs.msg
=======
>>>>>>> 7387faa... yes
from geometry_msgs.msg import Twist
import rospy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()s
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
<<<<<<< HEAD
        # TODO: Create publisher
        self.pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size = 10)
        #rospy.init_node('base', anonymous = True)
=======
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
>>>>>>> 7387faa... yes

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            line ar_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
<<<<<<< HEAD
        # TODO: Create Twist msg
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.pub.publish(msg)
        # TODO: Fill out msg
        # TODO: Publish msg
=======
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed 
        self.pub.publish(msg)
        # TODO: Fill out msg
        # TODO: Publish msg
        # rospy.logerr('Not implemented.')
>>>>>>> 7387faa... yes

    def stop(self):
        """Stops the mobile base from moving.
        """
<<<<<<< HEAD
        # TODO: Publish 0 velocity
        self.move(0,0)

=======
        self.move(0,0)
        # rospy.logerr('Not implemented.')
>>>>>>> 7387faa... yes
