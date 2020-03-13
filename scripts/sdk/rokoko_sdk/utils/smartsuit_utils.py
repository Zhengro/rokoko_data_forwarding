#!/usr/bin/env python

class SmartsuitFrame(object):
    def __init__(self, smartsuit_name=None, sensor_data=None):
        self.smartsuit_name = smartsuit_name
        self.sensor_data = sensor_data
    
    # original name 'print'
    def print_info(self):
        print('Smartsuit name: ',self.smartsuit_name)
        print('Sensor count: ',len(self.sensor_data))
        for sensor in self.sensor_data:
            print('Sensor address: ',sensor.getJointMapping(), sensor.sensor_address)
            print('Acceleration: ',sensor.acceleration)
            print('Quarternion: ',sensor.quarternion)
            print('Gyroscope: ',sensor.gyroscope)
            print('Position: ',sensor.position)
            print('Timestamp: ',sensor.timestamp)

class SensorData(object):
    def __init__(self, sensor_address=None, acceleration=None, gyroscope=None, quaternion=None, position=None, timestamp=None):
        self.sensor_address = sensor_address
        self.acceleration = acceleration
        self.gyroscope = gyroscope
        self.quaternion = quaternion
        self.position = position
        self.timestamp = timestamp
    
    def getJointMapping(self):
        if(self.sensor_address == 160): return 'SENSOR_HIP'
        if(self.sensor_address == 161): return 'SENSOR_STOMACH'
        if(self.sensor_address == 162): return 'SENSOR_CHEST'
        if(self.sensor_address == 163): return 'SENSOR_NECK'
        if(self.sensor_address == 64): return 'SENSOR_HEAD'
        if(self.sensor_address == 1): return 'SENSOR_LEFT_UPPER_LEG'
        if(self.sensor_address == 2): return 'SENSOR_LEFT_LOWER_LEG'
        if(self.sensor_address == 3): return 'SENSOR_LEFT_FOOT'
        if(self.sensor_address == 33): return 'SENSOR_LEFT_SHOULDER'
        if(self.sensor_address == 34): return 'SENSOR_LEFT_UPPER_ARM'
        if(self.sensor_address == 35): return 'SENSOR_LEFT_LOWER_ARM'
        if(self.sensor_address == 36): return 'SENSOR_LEFT_HAND'
        if(self.sensor_address == 97): return 'SENSOR_RIGHT_SHOULDER'
        if(self.sensor_address == 98): return 'SENSOR_RIGHT_UPPER_ARM'
        if(self.sensor_address == 99): return 'SENSOR_RIGHT_LOWER_ARM'
        if(self.sensor_address == 100): return 'SENSOR_RIGHT_HAND'
        if(self.sensor_address == 129): return 'SENSOR_RIGHT_UPPER_LEG'
        if(self.sensor_address == 130): return 'SENSOR_RIGHT_LOWER_LEG'
        if(self.sensor_address == 131): return 'SENSOR_RIGHT_FOOT'
