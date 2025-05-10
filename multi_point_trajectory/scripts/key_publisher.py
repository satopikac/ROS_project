#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import select
import termios

class KeyPoller():
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.new = termios.tcgetattr(self.fd)
        self.old = termios.tcgetattr(self.fd)
        self.new[3] = self.new[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new)
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)

    def poll(self, timeout=0.1):
        dr, dw, de = select.select([sys.stdin], [], [], timeout)
        if not dr:
            return None
        return sys.stdin.read(1)

def key_publisher():
    rospy.init_node('key_publisher', anonymous=True)
    pub = rospy.Publisher('/keyboard_input', String, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    rospy.loginfo("Use keys: [c] Circle, [s] Sinusoid, [l] Straight Line, [+] Speed Up, [-] Speed Down")

    with KeyPoller() as key_poller:
        while not rospy.is_shutdown():
            key = key_poller.poll()
            if key:
                if key in ['c', 's', 'l', '+', '-']:
                    msg = String()
                    msg.data = key
                    pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        key_publisher()
    except rospy.ROSInterruptException:
        pass