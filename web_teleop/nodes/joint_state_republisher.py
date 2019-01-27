#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    torso_pubs = []
    reader = JointStateReader()
    for joint in reader.jointname:
        torso_pubs.append(rospy.Publisher(
            'joint_state_republisher/' + joint, Float64, queue_size=10))
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(len(torso_pubs)):
            joint_state = reader.get_joint(reader.jointname[i])
            torso_pubs[i].publish(joint_state)
        rate.sleep()


if __name__ == '__main__':
    main()
