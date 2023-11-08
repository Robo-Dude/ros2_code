import pyrealsense2 as rs
import numpy as np
import cv2
from .submodules.paranoma import VideoStitcher
from .submodules.detection import Detect
import pyfakewebcam
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraPublisher(Node):
    
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # ...from Camera 1
        self.pipeline_1 = rs.pipeline()
        self.config_1 = rs.config()
        self.config_1.enable_device('239222302533')
        self.config_1.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config_1.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        
        # ...from Camera 2
        self.pipeline_2 = rs.pipeline()
        self.config_2 = rs.config()
        self.config_2.enable_device('336222300965')
        self.config_2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config_2.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start streaming from both cameras
        self.pipeline_1.start(self.config_1)
        self.pipeline_2.start(self.config_2)

        self.stitcher = VideoStitcher()
        self.detect = Detect()
        self.width, self.height = 1280, 720

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.fake_cam = pyfakewebcam.FakeWebcam("/dev/video14", self.width, self.height)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        ###################################################################################

        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_1 = self.pipeline_1.wait_for_frames()
        frames_1 = self.align.process(frames_1)
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()

        # Camera 2
        # Wait for a coherent pair of frames: depth and color
        frames_2 = self.pipeline_2.wait_for_frames()
        frames_2 = self.align.process(frames_2)
        depth_frame_2 = frames_2.get_depth_frame()
        color_frame_2 = frames_2.get_color_frame()

        ########################################################################################

        # Convert images to numpy arrays
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)

        # Convert images to numpy arrays
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_2, alpha=0.5), cv2.COLORMAP_JET)

        ###########################################################################################################

        color_image_1, boxes_id_1, centroid_box_1 = self.detect.detection(color_image_1, 0, None)

        color_image_2, boxes_id_2, centroid_box_2  = self.detect.detection(color_image_2, 0, None)

        ############################################################################################################

        color_image_1 = cv2.cvtColor(color_image_1, cv2.COLOR_BGR2RGB)

        color_image_2 = cv2.cvtColor(color_image_2, cv2.COLOR_BGR2RGB)

        result_image = self.stitcher.run(color_image_1, color_image_2)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)

        # cv2.namedWindow('Paranoma', cv2.WINDOW_NORMAL)
        # cv2.imshow('Paranoma', result_image)

        result_image = cv2.copyMakeBorder(result_image, 1000, 1000, 0, 0, cv2.BORDER_CONSTANT, None, value = 20) 

        # result_image = cv2.resize(result_image, (1280, 720))
        self.fake_cam.schedule_frame(result_image)

        #############################################################################################

        # Stack all images horizontally
        images1 = np.hstack((color_image_1, color_image_2))
        images2 = np.hstack((depth_colormap_1, depth_colormap_2))
        im_v = cv2.vconcat([images1, images2]) 

        # Show images from both cameras
        cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
        cv2.imshow('RealSense', im_v)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


