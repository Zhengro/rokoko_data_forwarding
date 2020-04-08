#!/usr/bin/env python

import rospy
import numpy as np
from rokoko_data_forwarding.msg import Suit


def callback(data):
    
    global data_frames
    global num_frames

    rospy.loginfo('Received one frame: \ntimestamp: {}\ntime: {}.'.format(data.timestamp, rospy.get_time()))
    print('\n')

    data_frames.append(list(data.frame.data))
    num_frames -= 1
    print(num_frames)

    if num_frames == 0:

        print('Total number of frames in this record: {}'.format(len(data_frames)))

        f = open(file_name, 'a')  
	f.write(str(data_frames)) 
	f.write('\n')
	f.close()

        print('Saved One Record!')

        rospy.signal_shutdown('Done.')

    # frame = np.asarray(data.frame.data)
    # frame_array = np.reshape(frame, (data.frame.layout.dim[0].size, data.frame.layout.dim[1].size))
    # print(frame_array.shape)
    # print('\n')


def listener():

    rospy.init_node('online_data_subscriber', anonymous=True, disable_signals=True)
    rospy.Subscriber('suit_online_data', Suit, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    data_frames = []
    num_frames = 600
    file_name = '6_600f_test.txt'
    listener()

    f = open(file_name, 'r')
    fl = f.readlines()
    c = 0
    for l in fl:
       c += 1
       l2 = eval(l)
    print('There are {} records of {} stored in {}.'.format(c, type(l2), file_name))
    f.close()
