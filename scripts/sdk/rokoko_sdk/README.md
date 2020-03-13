# Get started

- Setup Rokoko Studio to stream data. Set the ip to `127.0.0.1` and port to whatever you wish.
- Open `studio-sdk/run_sdk.py` and change the port on line 5
- Run the script

# Examples

## Default

Default example: [run_sdk.py](studio-sdk/run_sdk.py)

## MQTT

MQTT example with paho-mqtt: [run_mqtt_client.py](studio-sdk/run_mqtt_client.py)

# Reference system

The reference system used by the hub and the orientation and position of the sensors that will be received in the data frames are following right handed system with -Z Up and -Y Forward direction.

To translate the orientation of the sensors to the orientation of the joints of a specific character, we can use the following FBX character which has the correct joint orientations for the sensors 1-1 binding (after the Smartsuit reference system has been converted to the specific application reference system).

[RokokoGuyScalable.FBX](fbx/RokokoGuyScalable.FBX)

If we want to use a different rig we can still use a reference pose (prefered is T-pose) and add the joint orientation difference (for each joint) between the different rig and the RokokoGuy rig to the quaternion.


# Confidental

This repository is protected under NDA. See [LICENSE](LICENSE.md) for more info.