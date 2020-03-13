#!/usr/bin/env python

from studio_sdk import StudioSDK
import json
import paho.mqtt.client as paho

if __name__== "__main__":
    # Setup SDK
    sdk = StudioSDK('127.0.0.1', 14074)
    sdk.connect()

    # Setup MQTT client
    mqtt_broker = '127.0.0.1'
    mqtt_client = paho.Client('studio-sdk')
    mqtt_client.connect(mqtt_broker)
    
    while 1:
        data = sdk.receive()
        print('Received frame')
        smartsuit_frame = sdk.getProcessedFrame(data)
        mqtt_client.publish('smartsuit/data', json.dumps(smartsuit_frame, default=lambda x: x.__dict__))
        if not data: break
