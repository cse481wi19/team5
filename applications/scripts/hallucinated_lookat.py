#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
import rospy
import tf


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self, tf_listener):
        self.markers = []
        self._tf_listener = tf_listener

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node("hallucinated_lookat")
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
                                                                               
    reader = ArTagReader(tf.TransformListener())
    rospy.sleep(0.5)
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback, queue_size=10) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0: 
        rospy.sleep(0.1)

    head = robot_api.FullBodyLookAt(tf_listener=reader._tf_listener)
    sp = PointStamped()
    for marker in reader.markers:
        # TODO: get the pose to move to
        sp.header = marker.header
        sp.point = marker.pose.pose.position
        # print(sp)
        error = head.look_at(sp)
        if error:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            # return
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    rospy.logerr('Failed to move to any markers!')


if __name__ == '__main__':
    main()