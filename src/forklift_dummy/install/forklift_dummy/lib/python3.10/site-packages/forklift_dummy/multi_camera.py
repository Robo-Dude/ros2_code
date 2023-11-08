import pyrealsense2 as rs
import numpy as np
import cv2
from paranoma import VideoStitcher

# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('239222302533')
config_1.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('336222300965')
config_2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config_2.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming from both cameras
pipeline_1.start(config_1)
pipeline_2.start(config_2)

stitcher = VideoStitcher()

try:
    align_to = rs.stream.color
    align = rs.align(align_to)

    while True:
        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        frames_1 = align.process(frames_1)
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        if not depth_frame_1 or not color_frame_1:
            continue

        # Camera 2
        # Wait for a coherent pair of frames: depth and color
        frames_2 = pipeline_2.wait_for_frames()
        frames_2 = align.process(frames_2)
        depth_frame_2 = frames_2.get_depth_frame()
        color_frame_2 = frames_2.get_color_frame()

        if not depth_frame_2 or not color_frame_2:
            continue

        # Convert images to numpy arrays
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)

        # Convert images to numpy arrays
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        
        panorama = stitcher.run(color_image_1, color_image_2)
        # result_image = cv2.cvtColor(panorama, cv2.COLOR_BGR2RGB)
        # result_image = cv2.resize(result_image, (2460, 720))
        # fake_cam.schedule_frame(result_image)
        cv2.namedWindow('panorama', cv2.WINDOW_NORMAL)
        cv2.imshow('panorama', panorama)
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_2, alpha=0.5), cv2.COLORMAP_JET)

        # Stack all images horizontally
        images1 = np.hstack((color_image_1, color_image_2))
        images2 = np.hstack((depth_colormap_1, depth_colormap_2))
        im_v = cv2.vconcat([images1, images2]) 

        # Show images from both cameras
        cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
        cv2.imshow('RealSense', im_v)
        cv2.waitKey(1)

        # Save images and depth maps from both cameras by pressing 's'
        ch = cv2.waitKey(25)
        if ch==115:
            cv2.imwrite("my_image_1.jpg",color_image_1)
            cv2.imwrite("my_depth_1.jpg",depth_colormap_1)
            cv2.imwrite("my_image_2.jpg",color_image_2)
            cv2.imwrite("my_depth_2.jpg",depth_colormap_2)
       
finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()