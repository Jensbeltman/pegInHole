import pyrealsense2 as rs
import numpy as np
import cv2 as cv2
import open3d.open3d as o3d
from enum import Enum 
from os import makedirs
from os.path import exists, join 
import sys, time, threading, json, shutil, time


class PresetPaths():
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
    def __init__(self, path_preset=PresetPaths.ShortRangePreset, path_output='./dataset/',width=1280, height=720, fps= 15, clipDist = 1,init3DVis = True):
        # Define paths where data should be saved
        self.path_output = path_output
        self.path_color =  join(self.path_output, "depth")
        self.path_depth =  join(self.path_output, "color")
        self.make_clean_folder(self.path_output)
        self.make_clean_folder(self.path_color)
        self.make_clean_folder(self.path_depth)
        self.path_preset = path_preset

        self.saveFrames = False
        self.saveFrame = False

        
        # Variable for data storage
        self.pcd = o3d.geometry.PointCloud()
        self.color_img = None
        self.pcdVis_frame_count = 0
        self.frame_count = 0
        self.saved_frame_count = 0

        # Create a pipeline camera pipeline object
        self.pipeline = rs.pipeline()

        #Create a config and configure the pipeline to stream
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

        # Start streaming based on config
        self.profile = self.pipeline.start(self.config)
        self.on =  True

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # We will not display the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = clipDist  # 3 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        # Using preset HighAccuracy for recording
        #depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        rs.rs400_advanced_mode(self.profile.get_device()).load_json(loadPreset(self.path_preset))
        
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align = rs.align(rs.stream.color)

        self.flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

        # Visualization'
        if init3DVis:
            self.pcdVis = o3d.visualization.VisualizerWithKeyCallback()
            self.pcdVis.create_window()
            self.pcdVis.register_key_callback(32,self.toggleSaveFrames)

            self.pcdVis.register_key_callback(81,self.stopCamera)
            self.pcdVis.register_key_callback(83,self.saveOneFrame)

            # Get a frame before calling the vizualiser
            self.getFrames()
            self.pcdVis.add_geometry(self.pcd)
            self.visPCD()
        else:
            self.pcdVis = None

    def toggleSaveFrames(self,vis):
        self.saveFrames = not self.saveFrames

    def saveOneFrame(self,vis):
        self.saveFrame = True
    
    def stopCamera(self,vis):
        self.on = False

    def close(self):
        self.pipeline.stop()
        self.pcdVis.close()

    def visPCD(self):
        self.pcdVis.update_geometry(self.pcd)
        self.updateVis()

    def updateVis(self):
        self.pcdVis.poll_events()
        self.pcdVis.update_renderer()

    @staticmethod
    def make_clean_folder(path_folder):
        if not exists(path_folder):
            makedirs(path_folder)
        else:
            user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
            if user_input.lower() == 'y':
                shutil.rmtree(path_folder)
                makedirs(path_folder)
            else:
                exit()

       
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

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            return False

        depth_image_raw = np.array(aligned_depth_frame.get_data())
        color_image_raw = np.asarray(color_frame.get_data())
        depth_image = o3d.geometry.Image(depth_image_raw)
        color_image = o3d.geometry.Image(color_image_raw)

        if self.saveFrames or self.saveFrame:
                cv2.imwrite("%s/%06d.png" % \
                        (self.path_depth, self.saved_frame_count), depth_image_raw)
                cv2.imwrite("%s/%06d.jpg" % \
                        (self.path_color, self.saved_frame_count), color_image_raw)
                print("Saved color + depth image %06d" % self.saved_frame_count)
                self.saved_frame_count += 1    
                self.saveFrame = False   

        # Generate Data for visualizer
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image,
            depth_image,
            depth_scale=1.0 / self.depth_scale,
            depth_trunc=self.clipping_distance_in_meters,
            convert_rgb_to_intensity=False)

        intrinsics = self.get_intrinsic_matrix(color_frame)

        temp = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsics)
        temp.transform(self.flip_transform)
        self.pcd.points = temp.points
        self.pcd.colors = temp.colors

        return frames != None

    


if __name__ == "__main__":
    camera = CameraRGBD(init3DVis=True,fps=15,path_output='./Data/')
    while camera.on:
        tS = time.time()
        camera.getFrames()
        camera.visPCD()
        tE = time.time()
        #print(1/(tE-tS))
    camera.close()
    print('Camera turned off')



        
        