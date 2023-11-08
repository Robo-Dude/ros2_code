import pyrealsense2 as rs
import math
import time

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_device('336222300965')
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p

p = initialize_camera()
gt = 0
roll = 0.0
pitch = 0.0
yaw = 0.0
first = True
alpha = 0.98

try:
    while True:
        f = p.wait_for_frames()

        #gather IMU data
        accel = f[0].as_motion_frame().get_motion_data()
        gyro = f[1].as_motion_frame().get_motion_data()

        ts = time.time() 

        dt = ts - gt

        roll += gyro.x * dt
        pitch += gyro.y * dt
        yaw += gyro.z * dt

        # Apply complementary filter to combine accelerometer and gyroscope data
        roll_acc = math.atan2(accel.x, math.sqrt(accel.y ** 2 + accel.z ** 2))
        pitch_acc = math.atan2(accel.y, math.sqrt(accel.x ** 2 + accel.z ** 2))

        roll = alpha * roll + (1 - alpha) * roll_acc
        pitch = alpha * pitch + (1 - alpha) * pitch_acc

        # Convert angles to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        print(f"Roll: {roll_deg:.2f} degrees, Pitch: {pitch_deg:.2f} degrees, Yaw: {yaw_deg:.2f} degrees")

        # Simulated time step
        gt = dt
        time.sleep(dt)

finally:
    p.stop()
