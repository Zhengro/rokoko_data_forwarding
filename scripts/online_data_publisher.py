#!/usr/bin/env python

import rospy
from forward_suit_online_data.msg import Suit
from std_msgs.msg import MultiArrayDimension
from rokoko_sdk.studio_sdk import StudioSDK
import time
import numpy as np


def process_one_frame(one_frame):
    """
    Create a list to store 13 paras for each sensor in a frame.
    :param one_frame: one frame from the smartsuit
    :return: data_list: the created list
             timestamp: the timestamp of that frame
    """
  
    sensor_count = 19
    timestamp = one_frame.sensor_data[0].timestamp
    data_list = []

    for i in range(sensor_count):
        
        # acceleration
        data_list.append(one_frame.sensor_data[i].acceleration.x)
        data_list.append(one_frame.sensor_data[i].acceleration.y)
        data_list.append(one_frame.sensor_data[i].acceleration.z)
        # gyroscope
        data_list.append(one_frame.sensor_data[i].gyroscope.x)
        data_list.append(one_frame.sensor_data[i].gyroscope.y)
        data_list.append(one_frame.sensor_data[i].gyroscope.z)
        # magnetometer
        data_list.append(one_frame.sensor_data[i].position.x)
        data_list.append(one_frame.sensor_data[i].position.y)
        data_list.append(one_frame.sensor_data[i].position.z)
        # quaternion
        data_list.append(one_frame.sensor_data[i].quaternion.x)
        data_list.append(one_frame.sensor_data[i].quaternion.y)
        data_list.append(one_frame.sensor_data[i].quaternion.z)
        data_list.append(one_frame.sensor_data[i].quaternion.w)

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

    sdk = StudioSDK(ip, port)
    sdk.connect()
    start_time = time.time()
    while not rospy.is_shutdown():
        byte = sdk.receive()
        smartsuit_frame = sdk.getProcessedFrame(byte)

        if time.time() - start_time > warm_up:
            data_list, timestamp = process_one_frame(smartsuit_frame)

            msg_data = Suit()
	    msg_data.frame.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
            msg_data.frame.layout.dim[0].label = 'sensor_count' 
	    msg_data.frame.layout.dim[0].size = 19
	    msg_data.frame.layout.dim[0].stride = 19*13
	    msg_data.frame.layout.dim[1].label = 'num_paras' 
	    msg_data.frame.layout.dim[1].size = 13
	    msg_data.frame.layout.dim[1].stride = 13
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
