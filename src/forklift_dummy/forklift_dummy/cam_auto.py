import pyrealsense2 as rs
import numpy as np
import cv2
from .submodules.centroidtracker import CentroidTracker
import pyfakewebcam
import rclpy
from rclpy.node import Node
from message_interfaces.msg import State , Coordinate, Imu
import torch

class CameraPublisher(Node):
    
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_coordinate = self.create_publisher(Coordinate, '/coordinate', 100)
        self.publisher_imu = self.create_publisher(Imu, '/imu', 100)
        timer_period = 0.0001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            State,
            '/state',
            self.state_callback,
            100)
        self.subscription  # prevent unused variable warning
        
        # ...from Camera 1 front
        self.pipeline_front = rs.pipeline()
        self.config_front = rs.config()
        self.config_front.enable_device('336222300965')
        self.config_front.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config_front.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config_front.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        self.config_front.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        
        # ...from Camera 2 back
        self.pipeline_back = rs.pipeline()
        self.config_back = rs.config()
        self.config_back.enable_device('239222302533')
        self.config_back.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config_back.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start streaming from both cameras
        self.pipeline_front.start(self.config_front)
        self.pipeline_back.start(self.config_back)

        self.width, self.height = 1280, 720

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.k = None

        self.n = 0

        self.ct = CentroidTracker()

        self.rects = []
        self.boxes_id = []
        self.centroid_box = []
        self.active_id = 0
        self.active = 0
        self.locked_box = None
        self.n = 0
        self.reset = False
        self.target_z = 1.0
        self.target_x = 611.0
        self.target_y = 360.0
        self.paused = True
        self.reverse = False

        self.model = torch.hub.load(
            "/home/ros2/ros2_ws/src/forklift_dummy/forklift_dummy/submodules/yolov5",
            "custom",
            path="/home/ros2/ros2_ws/src/forklift_dummy/forklift_dummy/submodules/yolov5/final_best.pt",
            source="local",
        )  # local repo

        self.model.conf = 0.6  # confidence threshold (0-1)
        self.model.iou = 0.7  # NMS IoU threshold (0-1)

        self.fake_cam = pyfakewebcam.FakeWebcam("/dev/video20", 1280, 720)

    def state_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.k = msg.k
        self.input()

    def input(self):
        # print(self.k)

        if self.k == "l":
            if self.paused == False:
                if self.active == 0:
                    self.active = self.n - 1
                else:
                    self.active -= 1
                # print(self.active)
                # print(self.boxes_id)
                self.active_id = self.boxes_id[self.active]

        elif self.k == "r":
            if self.paused == False:
                if self.active == self.n - 1:
                    self.active = 0
                else:
                    self.active += 1
                self.active_id = self.boxes_id[self.active]

        elif self.k == "o":
            if self.paused == False:
                self.selected = self.active
                self.locked_box = self.boxes_id[self.selected]
                # print(self.locked_box)

        if self.k == "f":
            self.reverse = False

        if self.k == "x":
            self.reverse = True

        if self.k == "p":
            self.paused = False

        if self.k == "m":
            self.paused = True
            self.reset = True


    def plot_boxes(self, results, frame_p, depth_frame, depth_mat):
        # print(results)

        labels, cord = (
            results.xyxyn[0][:, -1].cpu().numpy(),
            results.xyxyn[0][:, :-1].cpu().numpy(),
        )
        n = len(labels)
        x_shape, y_shape = frame_p.shape[1], frame_p.shape[0]
        self.rects = []
        # print(cord)
        for i in range(len(labels)):
            row = cord[i]
            if row[4] >= 0.2:
                x1, y1, x2, y2 = (
                    int(row[0] * x_shape),
                    int(row[1] * y_shape),
                    int(row[2] * x_shape),
                    int(row[3] * y_shape),
                )
                self.rects.append([x1, y1, x2, y2])

        objects = self.ct.update(self.rects)

        # print(type(objects))
        # print(objects)

        self.boxes_id = []
        self.centroid_box = []

        for objectID, centroid in objects.items():
            # draw both the ID of the object and the centroid of the
            # object on the output frame
            text = "ID {}".format(objectID)
            self.boxes_id.append(objectID)
            self.centroid_box.append(centroid)
            cv2.putText(
                frame_p,
                text,
                (centroid[0] - 10, centroid[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.circle(frame_p, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        if len(self.boxes_id) > 0:
            # print(self.boxes_id)

            for i in self.boxes_id:
                bgr = (0, 0, 255)

                # print(self.boxes_id.index(i))
                # print(self.rects)
                # print(self.active_id)
                x = self.centroid_box[self.boxes_id.index(i)][0]
                y = self.centroid_box[self.boxes_id.index(i)][1]

                # print(self.locked_box)
                if len(self.rects) > 0:
                    for startX, startY, endX, endY in self.rects:
                        # use the bounding box coordinates to derive the centroid
                        cX = int((startX + endX) / 2.0)
                        cY = int((startY + endY) / 2.0)

                        if x == cX and y == cY:
                            x1 = startX

                            y1 = startY

                            x2 = endX

                            y2 = endY

                    if i == self.active_id:
                        bgr = (0, 255, 0)

                    if self.locked_box == i:
                        overlay = frame_p.copy()
                        cv2.rectangle(
                            overlay,
                            (x1, y1),
                            (x2, y2),
                            (
                                255,
                                0,
                                0,
                            ),
                            -1,
                        )
                        alpha = 0.4
                        frame_p = cv2.addWeighted(overlay, alpha, frame_p, 1 - alpha, 0)
                        self.target_z = depth_mat.get_distance(int(x1 + (x2 - x1)/2), int(y2 - (y2 - y1)/2))
                        # self.mid_point = (
                        #     f"{x1 + (x2 - x1)/2},{y2 - (y2 - y1)/2},{x1},{x2}"
                        # )
                        self.target_x = int(x1 + (x2 - x1)/2)
                        self.target_y = int(y2 - (y2 - y1)/2)

                    else:
                        cv2.rectangle(frame_p, (x1, y1), (x2, y2), bgr, 2)
                        cv2.rectangle(depth_frame, (x1, y1), (x2, y2), bgr, 2)

                # cv.putText(frame, f"Object{i}", (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 1)

        self.n = len(self.boxes_id)

        return frame_p , depth_frame
    

    def timer_callback(self):
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        ###################################################################################

        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_front = self.pipeline_front.wait_for_frames()
        frames_front = self.align.process(frames_front)
        depth_frame_front = frames_front.get_depth_frame()
        color_frame_front = frames_front.get_color_frame()

        # Camera 2
        # Wait for a coherent pair of frames: depth and color
        frames_back = self.pipeline_back.wait_for_frames()
        frames_back = self.align.process(frames_back)
        depth_frame_back = frames_back.get_depth_frame()
        color_frame_back = frames_back.get_color_frame()

        ########################################################################################

        accel_frame = frames_front.first_or_default(rs.stream.accel).as_motion_frame()
        gyros_frame = frames_front.first_or_default(rs.stream.gyro).as_motion_frame()

        ts = frames_front.get_timestamp()
  
        if accel_frame:
            accel_x = accel_frame.get_motion_data().x
            accel_y = accel_frame.get_motion_data().y
            accel_z = accel_frame.get_motion_data().z
        if gyros_frame:
            gyros_x = gyros_frame.get_motion_data().x
            gyros_y = gyros_frame.get_motion_data().y
            gyros_z = gyros_frame.get_motion_data().z

        ########################################################################################

        # Convert images to numpy arrays
        depth_image_front = np.asanyarray(depth_frame_front.get_data())
        color_image_front = np.asanyarray(color_frame_front.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_front = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_front, alpha=0.5), cv2.COLORMAP_JET)

        # Convert images to numpy arrays
        depth_image_back = np.asanyarray(depth_frame_back.get_data())
        color_image_back = np.asanyarray(color_frame_back.get_data())
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_back = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_back, alpha=0.5), cv2.COLORMAP_JET)
       
        ###########################################################################################################

        color_image_back = color_image_back

        if self.paused == True:

            if self.reverse == True:
                self.fake_cam.schedule_frame(color_image_back)
            else:
                color_image_front = cv2.cvtColor(color_image_front, cv2.COLOR_BGR2RGB)
                self.fake_cam.schedule_frame(color_image_front)

        else:

            img = cv2.cvtColor(color_image_front, cv2.COLOR_RGB2BGR)

            results = self.model(img)

            color_frame_final , depth_frame_final = self.plot_boxes(results, color_image_front , depth_colormap_front, depth_frame_front)

            depth_colormap_front = depth_frame_final

            color_frame_final = cv2.cvtColor(color_frame_final, cv2.COLOR_BGR2RGB)

            self.fake_cam.schedule_frame(color_frame_final)

        #############################################################################################

        # Stack all images horizontally
        # images1 = np.hstack((color_image_front, color_image_back))
        # images2 = np.hstack((depth_colormap_front, depth_colormap_back))
        # im_v = cv2.vconcat([images1, images2]) 

        # # Show images from both cameras
        # cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
        # cv2.imshow('RealSense', im_v)
        # cv2.waitKey(1)

        ##############################################################################################

        msg = Coordinate()
        msg.x = float(self.target_x)
        msg.y = float(self.target_y)
        msg.z = float(self.target_z)
        self.publisher_coordinate.publish(msg)

        msg_imu = Imu()
        msg_imu.timestamp = ts
        msg_imu.linear_acceleration.x = accel_x
        msg_imu.linear_acceleration.y = accel_y
        msg_imu.linear_acceleration.z = accel_z

        msg_imu.angular_velocity.x = gyros_x
        msg_imu.angular_velocity.y = gyros_y
        msg_imu.angular_velocity.z = gyros_z

        self.publisher_imu.publish(msg_imu)

        ##################################################################################################

        if self.reset == True:

            self.ct = CentroidTracker()
            self.k = None
            self.n = 0
            self.rects = []
            self.boxes_id = []
            self.centroid_box = []
            self.active_id = 0
            self.active = 0
            self.locked_box = None
            self.n = 0
            self.reset = False
            self.target_z = 1.0
            self.target_x = 611.0
            self.target_y = 360.0
            self.paused = True
            self.reverse = False 

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


