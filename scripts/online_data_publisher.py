#!/usr/bin/env python

import rospy
import time
import json
from rokoko_data_forwarding.msg import Suit
from std_msgs.msg import MultiArrayDimension
from custom_streaming_udp import Socket
from custom_streaming_usage import SuitComposition


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

    print('Forward one frame: \ntimestamp: {}\ntime: {}.\n'.format(timestamp, rospy.get_time()))

    return data_list, timestamp


def talker(ip, port, warm_up, fps):
    """
    Publish suit online data.
    :param ip: the ip address for forwarding data
    :param port: the selected port for forwarding data
    :param warm_up: n seconds waiting ahead of the actual streaming
    :return: None

    Must define a custom message named 'Suit' in advance, i.e.,
    std_msgs/Float64MultiArray frame
    float64 timestamp
    """

    pub = rospy.Publisher('suit_online_data', Suit, queue_size=10)
    rospy.init_node('online_data_publisher', anonymous=True)
    rate = rospy.Rate(fps)

    S = Socket(ip, port)
    S.connect()
    alarm_time = [i for i in range(1, warm_up+1)]
    alarm_idx = 0
    start_time = time.time()
    
    while not rospy.is_shutdown():
        data_byte = S.receive()               # <class 'bytes'> or False
        if not data_byte:
            S.disconnect()
            print('No data from Studio.')
            break
        data_str = data_byte.decode('ASCII')  # <class 'str'>
        data_json = (json.loads(data_str))    # <class 'dict'>

	if warm_up > 0 and time.time() - start_time > alarm_time[alarm_idx]:
            print('{}s'.format(warm_up))
            warm_up -= 1
            alarm_idx += 1

        if warm_up == 0:
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
            print('\n')
            pub.publish(msg_data)
            rate.sleep()


if __name__ == '__main__':
    try:
        # configure the ip and port
        ip = '192.168.0.140'
        port = 14043

        # set several seconds to ensure the steady streaming without burst in the first several frames
        warm_up = 10
        fps = 80

        talker(ip=ip, port=port, warm_up=warm_up, fps=fps)
    except rospy.ROSInterruptException:
        pass
