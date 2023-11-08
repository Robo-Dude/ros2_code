#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from message_interfaces.msg import Imu
import math

class Euler(Node):

    def __init__(self):

        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            100)
        self.subscription  # prevent unused variable warning

        self.ts = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.last_ts_gyro = 0

        # self.msg_angle = Angle()
        # self.msg_angle.euler_angle.x = 0.0
        # self.msg_angle.euler_angle.y = 0.0
        # self.msg_angle.euler_angle.z = 0.0

        # self.publisher_angle = self.create_publisher(Angle, '/euler', 100)
        # timer_period = 0.0001  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def imu_callback(self, msg_imu):

        self.ts = msg_imu.timestamp
        self.accel_x =  msg_imu.linear_acceleration.x
        self.accel_y = msg_imu.linear_acceleration.y
        self.accel_z =  msg_imu.linear_acceleration.z

        self.gyro_x = msg_imu.angular_velocity.x
        self.gyro_y = msg_imu.angular_velocity.y
        self.gyro_z = msg_imu.angular_velocity.z
        print(msg_imu)

    def timer_callback(self):

        if (first):
            first = False
            self.last_ts_gyro = self.ts

            # accelerometer calculation
            self.accel_angle_z = math.degrees(math.atan2(self.accel_y, self.accel_z))
            self.accel_angle_x = math.degrees(math.atan2(self.accel_x, math.sqrt(self.accel_y * self.accel_y + self.accel_z * self.accel_z)))
            self.accel_angle_y = math.degrees(math.pi)

        #calculation for the second frame onwards

        # gyrometer calculations
        self.dt_gyro = (self.ts - self.last_ts_gyro) / 1000
        self.last_ts_gyro = self.ts

        self.gyro_angle_x = self.gyro_x * self.dt_gyro
        self.gyro_angle_y = self.gyro_y * self.dt_gyro
        self.gyro_angle_z = self.gyro_z * self.dt_gyro

        self.dangleX = self.gyro_angle_x * 57.2958
        self.dangleY = self.gyro_angle_y * 57.2958
        self.dangleZ = self.gyro_angle_z * 57.2958

        self.totalgyroangleX = self.accel_angle_x + self.dangleX
        self.totalgyroangleY = self.accel_angle_y + self.dangleY
        self.totalgyroangleZ = self.accel_angle_z + self.dangleZ

        #accelerometer calculation
        self.accel_angle_z = math.degrees(math.atan2(self.accel.y, self.accel.z))
        self.accel_angle_x = math.degrees(math.atan2(self.accel.x, math.sqrt(self.accel.y * self.accel.y + self.accel.z * self.accel.z)))
        self.accel_angle_y = math.degrees(math.pi)

        #combining gyrometer and accelerometer angles
        combinedangleX = self.totalgyroangleX * self.alpha + self.accel_angle_x * (1-self.alpha)
        combinedangleZ = self.totalgyroangleZ * self.alpha + self.accel_angle_z * (1-self.alpha)
        combinedangleY = self.totalgyroangleY

        # print("Angle -  X: " + str(round(combinedangleX,2)) + "   Y: " + str(round(combinedangleY,2)) + "   Z: " + str(round(combinedangleZ,2)))

        self.msg_angle.euler_angle.x = round(combinedangleX,2)
        self.msg_angle.euler_angle.y = round(combinedangleY,2)
        self.msg_angle.euler_angle.z = round(combinedangleZ,2)
        
        self.publisher_angle.publish(self.msg_angle)

def main(args=None):
    rclpy.init(args=args)

    euler_subscriber = Euler()

    rclpy.spin(euler_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    euler_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  


