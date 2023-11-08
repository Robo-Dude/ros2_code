#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from message_interfaces.msg import State
from std_msgs.msg import Int16

class controllerPublisher(Node):

    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_pwm_one = self.create_publisher(Int16, '/pwm1', 10)
        self.publisher_pwm_two = self.create_publisher(Int16, '/pwm2', 10)

        self.subscription1 = self.create_subscription(State, '/state', self.listener_callback, 100)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(State, '/auto_state', self.autolistener_callback, 100)
        self.subscription2  # prevent unused variable warning
        
        self.msg1 = Int16()
        self.msg2 = Int16()
         
        self.msg1.data = 0
        self.msg2.data = 0

        self.axis_x = 0
        self.axis_y = 0

        self.pwm1 = 0
        self.pwm2 = 0

        self.reset = False

        self.paused = False

        self.k = False

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
       
        self.pwm1 = int(0.255 * self.axis_y - 0.255 * self.axis_x)
        self.pwm2 = int(0.255 * self.axis_y + 0.255 * self.axis_x)

        if self.pwm1 > 255 :
           self.pwm1 = 255
        elif self.pwm1 < -255:
           self.pwm1 = -255

        if self.pwm2 > 255 :
           self.pwm2 = 255
        elif self.pwm2 < -255:
           self.pwm2 = -255

        self.msg1.data = self.pwm1
        self.msg2.data = self.pwm2

        if self.paused == False:
            self.publisher_pwm_one.publish(self.msg1)
            self.publisher_pwm_two.publish(self.msg2)

        if self.reset == True:
            self.axis_x = 0
            self.axis_y = 0
            self.pwm1 = 0
            self.pwm2 = 0
            self.reset = False

    def listener_callback(self, msg):
     
        self.axis_x = msg.x
        self.axis_y = msg.y
        self.k = msg.k
        if self.k == "p":
            self.paused = True
        elif self.k == "m":
            self.paused = False
            self.reset = True

    def autolistener_callback(self, msg):
     
        self.k = msg.k
        if self.k == "p":
            self.paused = True
        elif self.k == "m":
            self.paused = False
            self.reset = True

def main(args=None):
    rclpy.init(args=args)

    controller_publisher = controllerPublisher()

    rclpy.spin(controller_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()