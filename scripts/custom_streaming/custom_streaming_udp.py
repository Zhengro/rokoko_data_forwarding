import socket
import select
import json


class Socket(object):
    """
    A Python interface for receiving online data from Rokoko Smartsuit Pro with Rokoko Studio Custom Streaming.
    """

    def __init__(self, receiver_ip=None, receiver_port=None, protocol=socket.SOCK_DGRAM, buffer_size=20000, timeout=10):
        """
        Create an instance of the socket API.
        :param receiver_ip: the IP address of the machine to which the data is forwarding
        :param receiver_port: the port of the machine to which the data is forwarding
        :param protocol: the default protocol is UDP
        :param buffer_size: the maximum amount of data to be received at once from the socket.
        :param timeout: time (in seconds) to wait to shutdown and close the socket if nothing is received from Studio
                        (typically when Custom Streaming is disabled manually).
        """
        self.receiver_address = (receiver_ip, receiver_port)
        self.receiver_socket = socket.socket(socket.AF_INET, protocol)
        self.buffer_size = buffer_size
        self.timeout = timeout

    def connect(self):
        """
        Connect to the receiver socket.
        """
        self.receiver_socket.bind(self.receiver_address)

    def receive(self):
        """
        Receive from the receiver socket.
        """
        ready_to_read, _, _ = select.select([self.receiver_socket, ], [], [], self.timeout)
        if len(ready_to_read) > 0:
            return self.receiver_socket.recv(self.buffer_size)
        else:
            return False

    def disconnect(self):
        """
        Shutdown and close the receiver socket.
        """
        print('Socket connection was broken.')
        self.receiver_socket.shutdown(2)  # 0 = done receiving; 1 = done sending; 2 = both
        self.receiver_socket.close()


if __name__ == "__main__":

    # first enable Custom Streaming in Rokoko Studio to stream data
    # then configure ip and port with the same setting in Custom Streaming
    ip = '192.168.0.153'
    port = 14043
    S = Socket(ip, port)
    S.connect()
    try:
        while 1:
            data_byte = S.receive()               # <class 'bytes'> or False
            if not data_byte:
                S.disconnect()
                print('No data from Studio.')
                break

            data_str = data_byte.decode('ASCII')  # <class 'str'>
            data_json = (json.loads(data_str))    # <class 'dict'>
            # print(json.dumps(data_json, indent=4))

            # print(data_json['timestamp'])               # valid timestamp
            # print(data_json['actors'][0]['timestamp'])  # always 0.0
    except KeyboardInterrupt:
        print('Ctrl+c was pressed.')
        S.disconnect()
