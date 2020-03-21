import pyrealsense2 as rs
import numpy as np
import cv2 as cv2
import open3d.open3d as o3d
from enum import Enum 
import json
import threading
import time
class Preset():
    BodyScanPreset = "./realsensePresets/BodyScanPreset.json"
    DefaultPreset_D415 = "./realsensePresets/DefaultPreset_D415.json"
    HandGesturePreset = "./realsensePresets/HandGesturePreset.jsonH"
    HighResMidDensityPreset = "./realsensePresets/HighResMidDensityPreset.json"
    LowResMidDensityPreset = "./realsensePresets/LowResMidDensityPreset.json"
    MidResMidDensityPreset = "./realsensePresets/MidResMidDensityPreset.json"
    custom = "./realsensePresets/custom.json"
    DefaultPreset_D435 = "./realsensePresets/DefaultPreset_D435.json"
    HighResHighAccuracyPreset = "./realsensePresets/HighResHighAccuracyPreset.json"
    LowResHighAccuracyPreset = "./realsensePresets/LowResHighAccuracyPreset.json"
    MidResHighAccuracyPreset = "./realsensePresets/MidResHighAccuracyPreset.json"
    ShortRangePreset = "./realsensePresets/ShortRangePreset.json"
    D415_RemoveIR = "./realsensePresets/D415_RemoveIR.json"
    EdgeMapD435 = "./realsensePresets/EdgeMapD435.json"
    HighResHighDensityPreset = "./realsensePresets/HighResHighDensityPreset.json"
    LowResHighDensityPreset = "./realsensePresets/LowResHighDensityPreset.json"
    MidResHighDensityPreset = "./realsensePresets/MidResHighDensityPreset.json"

def loadPreset(path):
    preset_string = ''
    with open(path) as json_file:
        preset_string = json_file.read()
    return preset_string

class CameraRGBD():
    def __init__(self, width=1280, height=720, fps= 30, clipDist = 1,initVis = True):
        self.pcd = o3d.geometry.PointCloud()


         # Create a pipeline
        self.pipeline = rs.pipeline()

        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # We will not display the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = clipDist  # 3 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        # Using preset HighAccuracy for recording
        #depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        rs.rs400_advanced_mode(self.profile.get_device()).load_json(loadPreset(Preset.ShortRangePreset))
        
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align = rs.align(rs.stream.color)

        self.flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

        self.color_img = None
        self.pcdVis_frame_count = 0


        # Visualization
        self.pcdVis = o3d.visualization.VisualizerWithKeyCallback()
        if initVis:
            self.initPCDVis()

    def initPCDVis(self):
        # Initialize visualization
        self.pcdVis.register_key_callback(32,CameraRGBD.key_action_callback)
        self.pcdVis.create_window()
        self.getFrames()
        self.visPCD()
        # self.pcdVis.run()

    def visPCD(self):
        if self.pcdVis_frame_count == 0:
            self.pcdVis.add_geometry(self.pcd)

        self.pcdVis.update_geometry(self.pcd)

        self.updateVis()
        self.pcdVis_frame_count += 1

    def updateVis(self):
        self.pcdVis.poll_events()
        self.pcdVis.update_renderer()
       
    @classmethod
    def key_action_callback(vis, action):
        print("n pressed")
        if action == 1:  # key down
            vis.poll_events()
            vis.update_renderer()
        return True

    @staticmethod
    def get_intrinsic_matrix(frame):
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        out = o3d.camera.PinholeCameraIntrinsic(640, 480, intrinsics.fx,
                                                intrinsics.fy, intrinsics.ppx,
                                                intrinsics.ppy)
        return out


    def getFrames(self):
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                self.get_intrinsic_matrix(color_frame))

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                return False

            depth_image = o3d.geometry.Image(
                np.array(aligned_depth_frame.get_data()))
            color_temp = np.asarray(color_frame.get_data())
            color_image = o3d.geometry.Image(color_temp)

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / self.depth_scale,
                depth_trunc=self.clipping_distance_in_meters,
                convert_rgb_to_intensity=False)
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, intrinsic)
            temp.transform(self.flip_transform)

            self.pcd.points = temp.points
            self.pcd.colors = temp.colors
            return True

    


if __name__ == "__main__":
    camera = CameraRGBD()
    #camera.pcdVis.run()
    while True:
        #camera.updateVis()
        pass
        