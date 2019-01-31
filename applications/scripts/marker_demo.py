#!/usr/bin/env python
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
import rospy
import numpy as np

class NavPath(object):
    def __init__(self, pub):
        self._pub = pub
        self._pre_point = Point(x=0,y=0,z=0)
        self._marker = Marker(
                type=Marker.LINE_STRIP,
                id=1,
                ns="path",
                frame_locked = False,
                lifetime=rospy.Duration(60),
                action=Marker.ADD,
                scale=Vector3(0.06, 0.06, 0),
                pose=Pose(Point(0,0,0),Quaternion(0,0,0,1)),
                header=Header(frame_id='odom', stamp = rospy.Time.now()),
                color=ColorRGBA(1.0, 0, 1.0, 0.8),
                points=[])
        # rospy.logerr(self._pre_point)

    def dist(self, p1, p2):
        dp = [p1.x - p2.x, p1.y - p2.y, p1.z - p2.z]
        # rospy.loginfo(np.linalg.norm(np.array(dp)))
        return np.linalg.norm(np.array(dp))

    def callback(self, msg):
        # rospy.loginfo(msg)
        now_points = msg.pose.pose.position
        pre_points = self._pre_point
        if self.dist(now_points, pre_points) > 0.2:
            self._pre_point = now_points
            self._marker.points.append(self._pre_point)
            # rospy.loginfo(self._marker.points)
            self._pub.publish(self._marker)



def wait_for_time():                                              
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
        type=Marker.TEXT_VIEW_FACING,
        id=0,
        lifetime=rospy.Duration(1000),
        pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
        scale=Vector3(0.06, 0.06, 0.06),
        header=Header(frame_id='base_link'),
        color=ColorRGBA(1.0, 0, 1.0, 0.8),
        text=text)
    marker_publisher.publish(marker)

def main():
    rospy.init_node('my_node')
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    nav_path = NavPath(marker_publisher)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, nav_path.callback)
    rospy.sleep(0.5)     
    wait_for_time()                                                        
    show_text_in_rviz(marker_publisher, 'Hello world!')
    rospy.spin()

if __name__ == '__main__':
    main()