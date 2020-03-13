#!/usr/bin/env python

import socket
import sys
import struct
from utils.math_utils import Quaternion, Vector3D
from utils.smartsuit_utils import SmartsuitFrame, SensorData

class StudioSDK(object):
    '''
    A Python interface for Rokoko Studio data streaming

    :param studio_id:
        The IP address of Rokoko Studio which is streaming the data.
    '''
    def __init__(self, 
        studio_ip=None, 
        studio_port=None):
        '''
        Create an instance of the API interface.
        '''
        self.server_address = (studio_ip, studio_port)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def connect(self):
        '''
        Connect to the socket
        '''
        self.server_socket.bind(self.server_address)

    def receive(self):
        '''
        Receive from the socket
        '''
        buffer_size = 2000
        return self.server_socket.recv(buffer_size)

    def getProcessedFrame(self, frame):
        '''
        Process and return a Smarsuit frame.
        '''
        sensor_count = int((len(frame) - 4) / 60)
        offset = 4
        smartsuit_name = frame[0:offset-1].decode("utf-8")

        sensor_data_list = []

        for i in range(0, sensor_count):
            header = struct.unpack_from('I', frame, offset)[0]; offset += 4
            sensor_address = (header & 0xff)

            acceleration_x = struct.unpack_from('f', frame, offset)[0]; offset += 4
            acceleration_y = struct.unpack_from('f', frame, offset)[0]; offset += 4
            acceleration_z = struct.unpack_from('f', frame, offset)[0]; offset += 4
            acceleration = Vector3D(acceleration_x, acceleration_y, acceleration_z)
            
            quaternion_w = struct.unpack_from('f', frame, offset)[0]; offset += 4
            quaternion_x = struct.unpack_from('f', frame, offset)[0]; offset += 4
            quaternion_y = struct.unpack_from('f', frame, offset)[0]; offset += 4
            quaternion_z = struct.unpack_from('f', frame, offset)[0]; offset += 4
            quaternion = Quaternion(quaternion_w, quaternion_x, quaternion_y, quaternion_z)

            gyroscope_x = struct.unpack_from('f', frame, offset)[0]; offset += 4
            gyroscope_y = struct.unpack_from('f', frame, offset)[0]; offset += 4
            gyroscope_z = struct.unpack_from('f', frame, offset)[0]; offset += 4
            gyroscope = Vector3D(gyroscope_x, gyroscope_y, gyroscope_z)
        
            position_x = struct.unpack_from('f', frame, offset)[0]; offset += 4
            position_y = struct.unpack_from('f', frame, offset)[0]; offset += 4
            position_z = struct.unpack_from('f', frame, offset)[0]; offset += 4
            position = Vector3D(position_x, position_y, position_z)

            timestamp = struct.unpack_from('I', frame, offset)[0]; offset += 4

            sensor_data = SensorData(
                sensor_address=sensor_address, 
                acceleration=acceleration, 
                gyroscope=gyroscope, 
                quaternion=quaternion, 
                position=position, 
                timestamp=timestamp)

            sensor_data_list.append(sensor_data)

        return SmartsuitFrame(smartsuit_name=smartsuit_name, sensor_data=sensor_data_list)

    def close(self):
        self.server_socket.close()
