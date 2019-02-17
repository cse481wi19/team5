#! /usr/bin/env python
import robot_api
import rospy
import tf



def wait_for_time():
        """Wait for simulated time to begin.
        """
        while rospy.Time().now().to_sec() == 0:
            pass

def main():
    rospy.init_node("tf_listener")
    listener = tf.TransformListener()
    wait_for_time()
    rospy.sleep(0.1)
    rate = rospy.Rate(1.0)
    print("rate", rate)
    base_link='base_link'
    camera_frame='/upward_looking_camera_link'
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(base_link, '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print("trans: ", trans)
        print("rot: ", rot)
        rate.sleep()

if __name__ == '__main__':
    main()