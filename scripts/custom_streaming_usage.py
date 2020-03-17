#!/usr/bin/env python

from custom_streaming_udp import Socket
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import time
import json


class SuitComposition(object):
    """
    The usage of sensors on Rokoko Smartsuit Pro.
    """

    def __init__(self):
        self.body_parts = ['head', 'neck', 'chest', 'spine', 'hip',                         # central part
                           'leftUpLeg', 'leftLeg', 'leftFoot',                              # left leg
                           'rightUpLeg', 'rightLeg', 'rightFoot',                           # right leg
                           'leftShoulder', 'leftUpperArm', 'leftLowerArm', 'leftHand',      # left arm
                           'rightShoulder', 'rightUpperArm', 'rightLowerArm', 'rightHand']  # right arm

        self.connections = [[0, 1], [1, 2], [2, 3], [3, 4],         # central part
                            [4, 5], [5, 6], [6, 7],                 # left leg
                            [4, 8], [8, 9], [9, 10],                # right leg
                            [1, 11], [11, 12], [12, 13], [13, 14],  # left arm
                            [1, 15], [15, 16], [16, 17], [17, 18]]  # right arm

    def draw_skeleton(self, socket_object, duration):
        """
        Draw skeleton using online data.
        :param socket_object: the socket to receive online data
        :param duration: the duration (in seconds) of visualizing skeleton
        """
        plt.ion()
        fig = plt.figure()
        ax = plt.axes(projection='3d')

        start_time = time.time()
        while 1:
            data_byte = socket_object.receive()   # <class 'bytes'> or False
            if not data_byte:
                socket_object.disconnect()
                print('No data from Studio.')
                break

            data_str = data_byte.decode('ASCII')  # <class 'str'>
            data_json = (json.loads(data_str))    # <class 'dict'>
            # print(json.dumps(data_json, indent=4))

            xs = []
            ys = []
            zs = []
            for body_part in self.body_parts:
                xs.append(data_json['actors'][0][body_part]['position']['x'])
                ys.append(data_json['actors'][0][body_part]['position']['y'])
                zs.append(data_json['actors'][0][body_part]['position']['z'])

            ax.clear()
            for connection in self.connections:
                xpoints = [xs[connection[0]],
                           xs[connection[1]]]
                ypoints = [ys[connection[0]],
                           ys[connection[1]]]
                zpoints = [zs[connection[0]],
                           zs[connection[1]]]
                # (x, z, y)
                ax.scatter3D(xpoints, zpoints, ypoints, marker='o', c='blue')
                ax.plot3D(xpoints, zpoints, ypoints, 'gray')
            for i in range(len(xs)):
                ax.text(xs[i], zs[i], ys[i], '%s' % (str(i)), size=7, zorder=1, color='k')

            ax.set_xlabel('X(m)')
            ax.set_ylabel('Y(m)')
            ax.set_zlabel('Z(m)')
            # print(data_json['timestamp'])               # valid timestamp
            # print(data_json['actors'][0]['timestamp'])  # always 0.0
            ax.set_title('Timestamp: {0:.4f} s'.format(time.time() - start_time))
            plt.pause(1e-6)
            if time.time() - start_time > duration:
                break
        plt.ioff()
        plt.show()


if __name__ == "__main__":

    # first enable Custom Streaming in Rokoko Studio to stream data
    # then configure ip and port with the same setting in Custom Streaming
    ip = '192.168.0.142'
    port = 14043

    S = Socket(ip, port)
    S.connect()

    suit = SuitComposition()
    duration = 20
    suit.draw_skeleton(S, duration=duration)


