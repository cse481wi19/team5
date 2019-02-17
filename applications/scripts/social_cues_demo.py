#! /usr/bin/env python

import robot_api
import rospy


def print_usage():
    # NOTE: We don't expect you to implement look_at for Kuri
    # But if you do, show us because that would be impressive ;)
    # `eyes`, naturally, is Kuri only.
    print 'Usage:'
    print ' RUN IN BASE DIRECTORY (need ./files)'
    print '    rosrun applications social_cues_demo.py <nod/shake/happy/sad/neutral>'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('social_cues_demo')
    wait_for_time()
    rospy.sleep(1)
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]
    cues = robot_api.Social_Cues()

    if command == 'nod':
        cues.nod_head()
    elif command == 'shake':
        cues.shake_head()
    elif command == 'happy':
        cues.express_happy()
    elif command == 'sad':
        cues.express_sad()
    elif command == 'neutral':
        cues.express_neutral()
    else:
        print_usage()


if __name__ == '__main__':
    main()
