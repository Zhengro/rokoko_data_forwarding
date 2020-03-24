#!/usr/bin/env python

import rospy
import numpy as np
from rokoko_data_forwarding.msg import Suit


def callback(data):

    rospy.loginfo('Received one frame: \ntimestamp: {}\ntime: {}.'.format(data.timestamp, rospy.get_time()))
    print('\n')
    # frame = np.asarray(data.frame.data)
    # frame_array = np.reshape(frame, (data.frame.layout.dim[0].size, data.frame.layout.dim[1].size))
    # print(frame_array.shape)
    # print('\n')


def listener():

    rospy.init_node('online_data_subscriber', anonymous=True)
    rospy.Subscriber('suit_online_data', Suit, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
