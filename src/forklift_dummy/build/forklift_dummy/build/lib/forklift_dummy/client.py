#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from message_interfaces.msg import State
import socket

class connect(Node):

    def __init__(self):
        super().__init__('client_publisher')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_host = '192.168.2.56'
        self.server_port = 12345  # Change this to the server's port
        self.client_socket.connect((self.server_host, self.server_port))
        print("Connected to the server.")
        self.msg = State()
        self.msg.x = 0
        self.msg.y = 0
        self.msg.k = ''
        self.publisher_ = self.create_publisher(State, '/state', 100)
        timer_period = 0.000001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        self.data_str = str(self.client_socket.recv(1024).decode().strip())
    
        if len(self.data_str.split(',')) == 3:
            axis_x , axis_y, key = self.data_str.split(',')
            self.msg.x = int(axis_x)
            self.msg.y = int(axis_y)
            self.msg.k = key

        self.data_str = " "
        self.publisher_.publish(self.msg)
        # self.get_logger().info('Publishing: "%s"' % self.msg)
    
def main(args=None):
    rclpy.init(args=args)
    client_publisher = connect()
    rclpy.spin(client_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
