#!/usr/bin/env python3

import rospy as ros
from std_msgs.msg import Int16
from random import random


def main():
    ros.init_node('talker')
    publisher = ros.Publisher('/chat', Int16, queue_size=10)
    rate = ros.Rate(8)
    ros.sleep(1)

    msg = Int16()
    msg.data = 0

    while (not ros.is_shutdown()):
        if random() < 0.995:
            msg.data += 1
        else:
            msg.data = 0
        publisher.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()

