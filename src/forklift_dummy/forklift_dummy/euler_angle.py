#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from message_interfaces.msg import Imu , Angle , State
import math
import time

class Euler(Node):

    def __init__(self):

        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            100)
        self.subscription  # prevent unused variable warning

        self.subscription1 = self.create_subscription(State, '/state', self.listener_callback, 100)
        self.subscription1  # prevent unused variable warning

        self.ts = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.last_ts_gyro = 0

        self.first = True

        self.gyro_offset = [0, 0, 0]  # Gyroscope bias

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.reset = False

        self.alpha = 0.98

        self.msg_angle = Angle()
        self.msg_angle.euler_angle.x = 0.0
        self.msg_angle.euler_angle.y = 0.0
        self.msg_angle.euler_angle.z = 0.0

        self.publisher_angle = self.create_publisher(Angle, '/euler', 100)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        # print("reset request1")
    
        k = msg.k
        if k == "p":
           self.reset = True

        if k == "m":
           self.reset = True

    def imu_callback(self, msg_imu):

        self.ts = msg_imu.timestamp
        self.accel_x =  msg_imu.linear_acceleration.x
        self.accel_y = msg_imu.linear_acceleration.y
        self.accel_z =  msg_imu.linear_acceleration.z

        self.gyro_x = msg_imu.angular_velocity.x
        self.gyro_y = msg_imu.angular_velocity.y
        self.gyro_z = msg_imu.angular_velocity.z
        # print(msg_imu)

    def timer_callback(self):

        ts = time.time()  # Current timestamp in seconds

        # Calculate time difference
        if self.first:
            self.first = False
            self.gyro_offset[0] = self.gyro_x 
            self.gyro_offset[1] = self.gyro_y 
            self.gyro_offset[2] = self.gyro_z 

        else:
            dt_gyro = ts - self.last_ts_gyro
            self.last_ts_gyro = ts

            # Calculate orientation changes from gyroscope data
            gyro_angle_x = self.gyro_x - self.gyro_offset[0]
            gyro_angle_y = self.gyro_y - self.gyro_offset[1]
            gyro_angle_z = self.gyro_z - self.gyro_offset[2]

            # Convert angular rates to degrees
            gyro_angle_x_deg = math.degrees(gyro_angle_x)
            gyro_angle_y_deg = math.degrees(gyro_angle_y)
            gyro_angle_z_deg = math.degrees(gyro_angle_z)

            # Integrate gyroscope data to update roll, pitch, and yaw
            self.roll += gyro_angle_x_deg * dt_gyro
            self.pitch += gyro_angle_y_deg * dt_gyro
            self.yaw += gyro_angle_z_deg * dt_gyro

            # Apply complementary filter with accelerometer data (similar to previous code)

            # Calculate accelerometer angles (as before)
            accel_angle_x = math.degrees(math.atan2(self.accel_x, math.sqrt(self.accel_y ** 2 + self.accel_z ** 2)))
            accel_angle_y = math.degrees(math.atan2(self.accel_y, math.sqrt(self.accel_x ** 2 + self.accel_z ** 2)))
            accel_angle_z = math.degrees(math.pi)

            # Combine the angles using the complementary filter
            alpha = 0.98  # Adjust the filter coefficient as needed
            combined_angle_x = alpha * (gyro_angle_x_deg + self.roll) + (1 - alpha) * accel_angle_x
            combined_angle_y = alpha * (gyro_angle_y_deg + self.pitch) + (1 - alpha) * accel_angle_y
            combined_angle_z = alpha * (gyro_angle_z_deg + self.yaw) + (1 - alpha) * accel_angle_z
            
            self.msg_angle.euler_angle.x = float(f"{combined_angle_x:.2f}")
            self.msg_angle.euler_angle.y = float(f"{combined_angle_z:.2f}")
            self.msg_angle.euler_angle.z = float(f"{combined_angle_y:.2f}")

            self.publisher_angle.publish(self.msg_angle)

        if self.reset == True:

            # print("working")
            self.ts = 0.0
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0
            self.gyro_x = 0.0
            self.gyro_y = 0.0
            self.gyro_z = 0.0
            self.last_ts_gyro = 0
            self.first = True
            self.gyro_offset = [0, 0, 0]  # Gyroscope bias
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.reset = False

def main(args=None):
    rclpy.init(args=args)

    euler_subscriber = Euler()

    rclpy.spin(euler_subscriber)

    euler_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  


