#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from message_interfaces.msg import State, Coordinate, Angle

class setpoint(Node):

    def __init__(self):
        super().__init__('client_publisher')

        self.kp = [500, 3, 0]

        self.ki = [0.0, 0.0, 0.0]

        self.kd = [0.0, 300, 0.0]

        self.error = [0.0 , 0.0, 0.0]

        self.Iterm = [0.0 , 0.0, 0.0]

        self.prev_error = [0.0, 0.0, 0.0]

        self.distance = 1

        self.mid_point = 611

        self.set_distance = 1

        self.locked_distance = 1

        self.steering_input = 0

        self.set_mid = 611

        self.count = 0

        self.set_angle = 0

        self.current_angle = 0

        self.start = False

        self.paused = True

        self.reset = False

        self.target_locked = False

        self.done = False

        self.start2 = False

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

        self.subscription3 = self.create_subscription(
            Angle,
            '/euler',
            self.euler_callback,
            100)
        self.subscription3  # prevent unused variable warning

    def state_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.k = msg.k
        self.input()

    def coordinate_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.distance = msg.z
        self.mid_point = msg.x

    def euler_callback(self, msg_angle):

        self.current_angle = msg_angle.euler_angle.z
        if self.start2 == True:
            self.locked_distance = self.current_angle
            self.start = False
        
    def input(self):
        # print(self.k)
        if self.k == "p":
            self.paused = False
            self.start2 = True

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

        self.error[2] = (self.set_angle - self.current_angle) * - 1

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

        ##################### PID algo Yaw #############################################
        
        self.Iterm[2] = (self.Iterm[2] + self.error[2]) * self.ki[2]

        self.out_pwm_range = self.kp[2] * self.error[2] + self.Iterm[2] + self.kd[2] * ( self.error[2] - self.prev_error[2])

        self.prev_error[2] = self.error[2]

        self.pwm_out_angle = self.limit(self.out_pwm_range, 1000, -1000)

        #################################################################################
        
        if (self.error[0] > 0.5 or self.error[0] < -0.5):
            self.start = True
            print("started")

        if self.start == True:

            if (self.mid_point > 1000 or self.mid_point < 200 or self.distance < self.locked_distance/2 or self.done == False):
                self.steering_input = int(self.pwm_out_align - self.pwm_out_angle) 
                self.done = True
                print("pushing")
            else:
                self.steering_input = int(self.pwm_out_align + self.pwm_out_angle) 
                print("passing")
            
            self.steering_input = self.limit(self.steering_input, 1000, -1000)

            if (self.error[0] < 0.5 and self.error[0] > -0.5) :
                print("Target_locked")
                self.steering_input = int(self.pwm_out_align) 
                if (self.error[1] < 10 and self.error[1] > -10) :
                    self.target_locked = True
                    self.msg.x = int(0)
                    self.msg.y = int(0)
                    self.msg.k = ''
                    self.publisher_.publish(self.msg)
                    print("Angle_locked")

            if self.target_locked == False:
                self.msg.x = int(self.pwm_out_dist)
                self.msg.y = int(self.steering_input)
                self.msg.k = ''
                self.publisher_.publish(self.msg)

            else:
                print(self.count)
                if self.count < 2500:
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
            self.mid_point = 611
            self.set_distance = 1
            self.locked_distance = 1
            self.set_mid = 611
            self.count = 0
            self.set_angle = 0
            self.current_angle = 0
            self.start = False
            self.paused = True
            self.reset = False
            self.target_locked = False
            self.start2 = False
            self.steering_input = 0
            self.done = False

def main(args=None):
    rclpy.init(args=args)
    setpoint_publisher = setpoint()
    rclpy.spin(setpoint_publisher)
    setpoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
