#!/usr/bin/env python3

import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import time

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)

prev_frame_time = 0
  
# used to record the time at which we processed current frame 
new_frame_time = 0

font = cv2.FONT_HERSHEY_SIMPLEX 
            
# org 
fps_org = (5, 5+20) 

# fontScale 
fontScale = 1

# Blue color in BGR 
color = (255, 255, 255) 

thickness = 2

while True:
    # Capture RealSense frame
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_mat = np.asanyarray(depth_image)
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # time when we finish processing for this frame 
    new_frame_time = time.time() 
  
    fps = 1/(new_frame_time-prev_frame_time) 
    prev_frame_time = new_frame_time 
  
    # converting the fps into integer 
    fps = int(fps) 
    color_image = cv2.putText(color_image, f'FPS: {fps}', fps_org, font,  
                    fontScale, (0,0,255), thickness, cv2.LINE_AA) 

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((color_image, depth_colormap))

    images = cv2.resize(images, (1280, 480))

    # Display the result with bounding boxes
    cv2.imshow('Object Detection', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
