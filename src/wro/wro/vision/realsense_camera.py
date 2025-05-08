# ------------------------------------------------
# Copyright (c) Roche Diagnostics International AG
# ------------------------------------------------
# This class represents the intel realsense camera
import os
import time

import numpy as np
import pyrealsense2 as rs

import cv2 as cv

try:
    from .camera import DepthCamera
    from .camera_capture import RealSenseCapture
except:
    from camera import DepthCamera
    from camera_capture import RealSenseCapture

class RealSenseCamera(DepthCamera):
    """
    Helper for getting images from the realsense camera for detecting objects
    """

    def __init__(self):
        super().__init__()
        print(f"Camera initializing...")

        self.pipeline = rs.pipeline()

        config = rs.config()

        config.enable_stream(
            rs.stream.color,
            1280, 720,
            rs.format.rgb8, 30
        )
        config.enable_stream(
            rs.stream.depth,
            1280, 720,
            rs.format.z16, 30
        )

        self.profile = self.pipeline.start(config)

        self.device = self.pipeline.get_active_profile().get_device()

    def shutdown(self):
        """
        Shutdown the realsense camera
        """
        if self.pipeline is not None:
            self.pipeline.stop()

    def capture(self) -> RealSenseCapture:
        """
        Takes one frame from the realsense camera and returns a color, a depth and a combine image. Additionally,
        the intrinsic matrix from the image is sent.
        Returns:
            RealSenseCapture object containing the color, depth and intrinsic matrix
        """
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        frames = self.pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())

        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv.cvtColor(color_image, cv.COLOR_RGB2BGR)

        depth_profile = depth_frame.get_profile()

        # get intrinsic from aligned image
        depth_intrinsics = rs.video_stream_profile(depth_profile).get_intrinsics()
        intrinsic_mat = np.array([[depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                                  [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                                  [0, 0, 1]])

        return RealSenseCapture(color_image, depth_image, intrinsic_mat)