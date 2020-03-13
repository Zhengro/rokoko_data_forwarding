#!/usr/bin/env python

import json
from studio_sdk import StudioSDK

def test_frame_processing():

    # Open binary file
    byte = ''
    with open('studio-sdk/tests/data/frame_P_NH2.bin', 'rb') as f:
        byte = f.read(1144)
        # print(byte)

    # Open JSON file
    json_data = ''
    with open('studio-sdk/tests/data/frame_P_NH2.json', 'rb') as j:
        json_data = json.load(j)

    sdk = StudioSDK(None, None)
    binary_frame = sdk.getProcessedFrame(byte)
    
    assert json_data['name'] == binary_frame.smartsuit_name
    assert json_data['sensors'][0]['addr'] == binary_frame.sensor_data[0].sensor_address
    assert json_data['sensors'][0]['microseconds'] == binary_frame.sensor_data[0].timestamp

    assert json_data['sensors'][0]['acceleration']['x'] == binary_frame.sensor_data[0].acceleration.x
    assert json_data['sensors'][0]['acceleration']['y'] == binary_frame.sensor_data[0].acceleration.y
    assert json_data['sensors'][0]['acceleration']['z'] == binary_frame.sensor_data[0].acceleration.z
    
    assert json_data['sensors'][0]['quaternion']['x'] == binary_frame.sensor_data[0].quarternion.x
    assert json_data['sensors'][0]['quaternion']['y'] == binary_frame.sensor_data[0].quarternion.y
    assert json_data['sensors'][0]['quaternion']['z'] == binary_frame.sensor_data[0].quarternion.z
    assert json_data['sensors'][0]['quaternion']['w'] == binary_frame.sensor_data[0].quarternion.w
    
    assert json_data['sensors'][0]['gyro']['x'] == binary_frame.sensor_data[0].gyroscope.x
    assert json_data['sensors'][0]['gyro']['y'] == binary_frame.sensor_data[0].gyroscope.y
    assert json_data['sensors'][0]['gyro']['z'] == binary_frame.sensor_data[0].gyroscope.z
        
    
