#! /usr/bin/env python3
import rospy
import lifting_mechanism
import sys, select, os
import tty, termios




def test_func():
    while True:
        status = lifting.check_stand()
        pos = lifting.get_pos()
    # print(status, pos)
        
        key = sys.stdin.read(1)
        if key == 'l':
            if pos:
                lifting.lower_stand()
            else:
                lifting.lift_stand()

        if (key == '\x03'):
                    break

        rospy.sleep(0.1)
    



rospy.init_node("Lifting_test_node")
lifting = lifting_mechanism.lifting_mechanism()
#lifting.run_safety()
while True:
    lifting.lift_stand()
#test_func()
rospy.spin()

def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
