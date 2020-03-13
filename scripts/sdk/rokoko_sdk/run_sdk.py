#!/usr/bin/env python

from studio_sdk import StudioSDK
import json

if __name__== "__main__":
    sdk = StudioSDK('192.168.1.55', 14041)
    sdk.connect()
    while 1:
        data = sdk.receive()
        print('Received frame')
        smartsuit_frame = sdk.getProcessedFrame(data)
        print(json.dumps(smartsuit_frame, default=lambda x: x.__dict__))
        if not data: break
