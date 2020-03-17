#!/usr/bin/env python

import rospy
from rokoko_data_forwarding.msg import Suit


def callback(data):
    rospy.loginfo('Received one frame: \ntimestamp: {}\ntime: {}.'.format(data.timestamp, rospy.get_time()))
    # print(len(data.frame.data))
    # print('\n')


def listener():
    rospy.init_node('online_data_subscriber', anonymous=True)
    rospy.Subscriber('suit_online_data', Suit, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
