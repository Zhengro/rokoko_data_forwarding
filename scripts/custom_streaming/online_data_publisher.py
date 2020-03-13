#!/usr/bin/env python

import rospy
from forward_suit_online_data.msg import Suit
from std_msgs.msg import MultiArrayDimension
from custom_streaming_udp import Socket
from custom_streaming_usage import SuitComposition
import time
import json


def process_one_frame(one_frame):
    """
    Create a list to store (x, y, z) for each sensor in a frame.
    :param one_frame: one frame from the smartsuit (a dictionary)
    :return: data_list: the created list
             timestamp: the timestamp of that frame
    """

    suit = SuitComposition()
    timestamp = one_frame['timestamp']
    data_list = []

    for body_part in suit.body_parts:
        data_list.append(one_frame['actors'][0][body_part]['position']['x'])
        data_list.append(one_frame['actors'][0][body_part]['position']['y'])
        data_list.append(one_frame['actors'][0][body_part]['position']['z'])

    print('Forward one frame: \ntimestamp: {}\ntime: {}.'.format(timestamp, rospy.get_time()))

    return data_list, timestamp


def talker(ip, port, warm_up):
    """
    Publish suit online data.
    :param ip: the ip address for forwarding data
    :param port: the selected port for forwarding data
    :param warm_up: n seconds waiting ahead of the actual streaming
    :return: None

    Must define a custom message named 'Suit' in advance, i.e.,
    std_msgs/Float64MultiArray frame
    int64 timestamp
    """

    pub = rospy.Publisher('suit_online_data', Suit, queue_size=10)
    rospy.init_node('online_data_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    S = Socket(ip, port)
    S.connect()
    start_time = time.time()
    while not rospy.is_shutdown():
        data_byte = S.receive()               # <class 'bytes'> or False
        if not data_byte:
            S.disconnect()
            print('No data from Studio.')
            break
        data_str = data_byte.decode('ASCII')  # <class 'str'>
        data_json = (json.loads(data_str))    # <class 'dict'>

        if time.time() - start_time > warm_up:
            data_list, timestamp = process_one_frame(data_json)

            msg_data = Suit()

            msg_data.frame.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
            msg_data.frame.layout.dim[0].label = 'sensor_count'
            msg_data.frame.layout.dim[0].size = 19
            msg_data.frame.layout.dim[0].stride = 19*3
            msg_data.frame.layout.dim[1].label = 'num_paras'  # (x, y, z)
            msg_data.frame.layout.dim[1].size = 3
            msg_data.frame.layout.dim[1].stride = 3
            msg_data.frame.data = data_list
            msg_data.timestamp = timestamp

            rospy.loginfo(msg_data)
            pub.publish(msg_data)
            rate.sleep()


if __name__ == '__main__':
    try:
        # configure the ip and port
        ip = '192.168.0.142'
        port = 14041

        # set several seconds to ensure the steady streaming without burst in the first several frames
        warm_up = 5

        talker(ip=ip, port=port, warm_up=warm_up)
    except rospy.ROSInterruptException:
        pass
