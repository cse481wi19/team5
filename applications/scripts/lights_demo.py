#! /usr/bin/env python

import robot_api
from mobile_base_driver.msg import ChestLeds
from mobile_base_driver.msg import Led
import rospy


def print_usage():
    print 'Usage: rosrun applications lights_demo.py off'
    print '       rosrun applications lights_demo.py on'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('gripper_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]
    lights = robot_api.Lights()

    if command == 'off':
        lights.off()
    elif command == 'on':
        lights.all_leds([lights.BLUE]*lights.NUM_LEDS)
    elif command == 'color':
        count = 0
        color = [lights.RED, lights.BLUE, lights.GREEN]
        while True:
            num = count % 3
            lights.put_pixels([color[num]]*lights.NUM_LEDS)
            count += 1  
    else:
        print_usage()


if __name__ == '__main__':
    main()
