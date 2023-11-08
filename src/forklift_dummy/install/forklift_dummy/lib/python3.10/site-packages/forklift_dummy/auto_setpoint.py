#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from message_interfaces.msg import State, Coordinate

class setpoint(Node):

    def __init__(self):
        super().__init__('client_publisher')

        self.kp = [500, 5, 40]

        self.ki = [0.0, 0.0, 0.0]

        self.kd = [0.0, 300, 100.0]

        self.error = [0.0 , 0.0, 0.0]

        self.Iterm = [0.0 , 0.0, 0.0]

        self.prev_error = [0.0, 0.0, 0.0]

        self.distance = 1

        self.mid_point = 640

        self.set_distance = 1

        self.set_mid = 640

        self.count = 0

        self.start = False

        self.paused = True

        self.reset = False

        self.target_locked = False

        self.msg = State()
        self.msg.x = 0
        self.msg.y = 0
        self.msg.k = ''
        self.publisher_ = self.create_publisher(State, '/auto_state', 100)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            State,
            '/state',
            self.state_callback,
            100)
        self.subscription  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            Coordinate,
            '/coordinate',
            self.coordinate_callback,
            100)
        self.subscription2  # prevent unused variable warning

    def state_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.k = msg.k
        self.input()

    def coordinate_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.distance = msg.z
        self.mid_point = msg.x

    def input(self):
        # print(self.k)
        if self.k == "p":
            self.paused = False

        if self.k == "m":
            self.paused = True
            self.reset = True

    def limit(self, input_value, max_value, min_value):

        if input_value > max_value:
            return max_value
        elif input_value < min_value:
            return min_value
        else:
            return input_value
        

    def timer_callback(self):

        ################### Error Calculation ###################################

        self.error[0] = (self.set_distance - self.distance) 

        self.error[1] = (self.set_mid - self.mid_point) * - 1

        # print(self.error[0])

        ################### PID algo Distance ####################################

        self.Iterm[0] = (self.Iterm[0] + self.error[0]) * self.ki[0]

        self.out_pwm_range = self.kp[0]* self.error[0] + self.Iterm[0] + self.kd[0] * ( self.error[0] - self.prev_error[0] )

        self.prev_error[0] = self.error[0]

        self.pwm_out_dist = self.limit(self.out_pwm_range, 1000, -1000) 

        ##################### PID algo coordinate ###################################
        
        self.Iterm[1] = (self.Iterm[1] + self.error[1]) * self.ki[1]

        self.out_pwm_range = self.kp[1] * self.error[1] + self.Iterm[1] + self.kd[1] * ( self.error[1] - self.prev_error[1])

        self.prev_error[1] = self.error[1]

        self.pwm_out_align = self.limit(self.out_pwm_range, 1000, -1000)

        ###################################################################################

        if (self.error[0] > 0.5 or self.error[0] < -0.5):

            self.start = True

        if self.start == True:

            if (self.error[0] < 0.5 and self.error[0] > -0.5) :
                if (self.error[1] < 5 and self.error[1] > -5) :
                    self.target_locked = True
                    self.msg.x = int(0)
                    self.msg.y = int(0)
                    self.msg.k = ''
                    self.publisher_.publish(self.msg)

        if self.target_locked == False:
            self.msg.x = int(self.pwm_out_dist)
            self.msg.y = int(self.pwm_out_align)
            self.msg.k = ''
            self.publisher_.publish(self.msg)

        else:
            print(self.count)
            if self.count < 3000:
                self.target_locked = True
                self.msg.x = int(-800)
                self.msg.y = int(0)
                self.msg.k = ''
                self.publisher_.publish(self.msg)
                self.count += 1

            else:
                self.target_locked = True
                self.msg.x = int(0)
                self.msg.y = int(0)
                self.msg.k = 'm'
                self.publisher_.publish(self.msg)
                self.reset = True

        ###################################################################################

        if self.reset == True:

            print("Reseting")
            self.error = [0.0 , 0.0, 0.0]
            self.Iterm = [0.0 , 0.0, 0.0]
            self.prev_error = [0.0, 0.0, 0.0]
            self.distance = 1
            self.mid_point = 640
            self.set_distance = 1
            self.set_mid = 640
            self.count = 0
            self.start = False
            self.paused = True
            self.reset = False
            self.target_locked = False
    
def main(args=None):
    rclpy.init(args=args)
    setpoint_publisher = setpoint()
    rclpy.spin(setpoint_publisher)
    setpoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
