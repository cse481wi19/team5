#!/usr/bin/env python

import robot_api
import rospy


import sys, select, termios, tty

msg = """
Control Your Kuri!
---------------------------
Moving around:
   q    w    e
   a    s    d

q/e: eyelids open/close
Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}

eyeBindings = {'q': -0.1, 'e': 0.1}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = 1
turn = 1


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('kuri_teleop_key')
    base = robot_api.Base()
    head = robot_api.Head(None)

    x = 0
    th = 0
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    cur_eye = 0
    try:
        print msg
        print vels(speed, turn)
        while (1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]
                count = 0

                print vels(speed, turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key in eyeBindings.keys():
                cur_eye += eyeBindings[key]
                if cur_eye > head.EYES_CLOSED:
                    cur_eye = head.EYES_CLOSED
                elif cur_eye < head.EYES_HAPPY:
                    cur_eye = head.EYES_HAPPY
                head.eyes_to(cur_eye)
                continue
            elif key == ' ':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                head.cancel()
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn
            base.move(control_speed, control_turn)
    except Exception as e:
        rospy.logerr('{}'.format(e))
    finally:
        base.stop()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
