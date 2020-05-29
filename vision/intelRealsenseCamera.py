import pyrealsense2 as rs
import numpy as np
import cv2 as cv2

import open3d.open3d as o3d
from enum import Enum 
from os.path import join 
import sys, time, threading, json
from utility import ReaderJSON, readCC, make_clean_folder
from threading import Thread, Event

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
    jsonObj = json.load(open(path))
    json_string = str(jsonObj).replace("'", '\"')        
    return json_string


class ColorVisualizer(Thread):
    def __init__(self,img,winName,color_vis_update_flag,shutdown_flag, save_frame_flag, save_frames_flag,fps=30,name='ColorVisualizer'):
        super().__init__()
        self.name = name
        self.shutdown_flag = shutdown_flag
        self.save_frame_flag = save_frame_flag
        self.save_frames_flag = save_frames_flag
        self.color_vis_update_flag =  color_vis_update_flag

        self.winName = winName
        self.img = img

        self.sleepT = int(1000/fps)
        self.keys = [27, 83, 115, 76, 108]
        self.lastKeyTime = 0
        
        
    def run(self):
        print("{} thread started".format(self.name))
        
        while not self.shutdown_flag.is_set():
            if self.color_vis_update_flag.is_set():
                cv2.imshow(self.winName,self.img)
                self.color_vis_update_flag.clear()

            key = cv2.waitKey(self.sleepT)
            if key != -1:
                t = time.time()
                if (t-self.lastKeyTime)>0.5:
                    if key == 27:
                        self.shutdown_flag.set()
                    elif key == 83 or key == 115:
                        self.save_frame_flag.set()
                    elif key == 76 or key == 108:
                        if self.save_frames_flag.is_set():
                            self.save_frames_flag.clear()
                        else:
                            self.save_frames_flag.set()
                    
                    self.lastKeyTime = t
            
                        
        cv2.destroyAllWindows()
        
        print("{} thread stopped".format(self.name))

class CameraRGBD():
    # Note: the fps is purely for visualization the fps is set based on the preset file same goes for width and height
    def __init__(self,path_camCal=None, path_output='./data/dataset/', path_camera_preset=PresetPaths.custom, width=1280, height=720, fps= 15, clipDist = 1,visualization=True):
        # Define paths where data should be saved
        self.path_output = path_output
        self.path_depth = join(self.path_output, "depth")
        self.path_color =  join(self.path_output, "color")

        make_clean_folder(self.path_output)
        make_clean_folder(self.path_depth)
        make_clean_folder(self.path_color)
        
        self.width = width
        self.height = height

        self.color_img = np.zeros((height,width,3),dtype=np.uint8)
        self.pcd = o3d.geometry.PointCloud()
        
        self.path_camera_preset = path_camera_preset
        self.cameraMatrix, self.distCoeffs = readCC(path_camCal)
        self.intrinsics = o3d.camera.PinholeCameraIntrinsic(self.width, self.height, self.cameraMatrix[0,0],
                                            self.cameraMatrix[1,1], self.cameraMatrix[0,2],
                                            self.cameraMatrix[1,2])

      
        self.pcdVis_frame_count = 0
        self.frame_count = 0
        self.saved_frame_count = 0     

        self.color_vis_update_flag = Event()
        self.save_frame_flag = Event()
        self.save_frames_flag = Event()
        self.shutdown_flag = Event()

        # ----- Reals sense camera setup -----
        self.setCameraPreset()
        
        
        self.pipeline = rs.pipeline() # Create a pipeline camera pipeline object
        self.profile = self.pipeline.start() # Start streaming based on config
        self.dev = self.profile.get_device()
        
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.dev.first_depth_sensor()
        self.depth_scale =  self.depth_sensor.get_depth_scale()

        # We will not display the background of objects more than clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = clipDist  # 3 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

        # Using preset HighAccuracy for recording
        #depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        
        # Create an align object for aligning depth with color.
        self.align = rs.align(rs.stream.color)
        # flip transform 
        self.flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

        # Initializing the 2d color visualization 
        self.visualization = visualization
        if visualization:
            self.colorVis = ColorVisualizer(self.color_img, 'image',self.color_vis_update_flag, self.shutdown_flag, self.save_frame_flag, self.save_frames_flag,fps=fps)
            self.colorVis.start()

    def setCameraPreset(self):
        DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07","0B3A"]

        def find_device_that_supports_advanced_mode() :
            ctx = rs.context()
            ds5_dev = rs.device()
            devices = ctx.query_devices();
            for dev in devices:
                if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                    if dev.supports(rs.camera_info.name):
                        print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
                    return dev
            raise Exception("No device that supports advanced mode was found")

        try:
            dev = find_device_that_supports_advanced_mode()
            advnc_mode = rs.rs400_advanced_mode(dev)
            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

            # Loop until we successfully enable advanced mode
            while not advnc_mode.is_enabled():
                print("Trying to enable advanced mode...")
                advnc_mode.toggle_advanced_mode(True)
                # At this point the device will disconnect and re-connect.
                print("Sleeping for 5 seconds...")
                time.sleep(5)
                # The 'dev' object will become invalid and we need to initialize it again
                dev = find_device_that_supports_advanced_mode()
                advnc_mode = rs.rs400_advanced_mode(dev)
                print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")
                    # Get each control's current value

            advnc_mode.load_json(loadPreset(self.path_camera_preset))
            print('Preset loaded')

        except Exception as e:
            print('error ')
            print(e)

    def stopCamera(self):
        self.shutdown_flag.set()
        self.on = False

    def close(self):
        self.shutdown_flag.set()
        if self.visualization:
            self.colorVis.join()
        self.pipeline.stop()
        

    def getFrames(self):
        if self.shutdown_flag.is_set():
            self.close()

        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            return False
        
        
        color_img_raw = np.asarray(color_frame.get_data())
        depth_image_raw = np.array(aligned_depth_frame.get_data())
        depth_image = o3d.geometry.Image(depth_image_raw)
        color_image = o3d.geometry.Image(color_img_raw)
        cv2.cvtColor(color_img_raw,cv2.COLOR_RGB2BGR,self.color_img)
        #cv2.copyTo(color_img_raw,None,self.color_img)

        if self.visualization:
            # Generate Data for visualizer
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / self.depth_scale,
                depth_trunc=self.clipping_distance_in_meters,
                convert_rgb_to_intensity=False)

            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, self.intrinsics)
            temp.transform(self.flip_transform)
            self.pcd.points = temp.points
            self.pcd.colors = temp.colors

            self.color_vis_update_flag.set()
    

            saveFrames = self.save_frames_flag.is_set()
            saveFrame = self.save_frame_flag.is_set()
            if saveFrames or saveFrame:
                o3d.io.write_point_cloud("%s/%06d.pcd" % (self.path_depth, self.saved_frame_count), self.pcd)
                cv2.imwrite("%s/%06d.jpg" % (self.path_color, self.saved_frame_count), self.color_img)
                print("Saved color + depth image %06d" % self.saved_frame_count)
                self.saved_frame_count += 1  

                if saveFrame:
                    self.save_frame_flag.clear() 
    

        return frames != None


if __name__ == "__main__":
    camera = CameraRGBD(path_camCal='./json/cam_cal_D415_1280x720.json',fps=15,path_output='./data/trData/')
    times = []
    while not camera.shutdown_flag.is_set():
        tS = time.time()
        camera.getFrames()
        tE = time.time()
        times.append(1/(tE-tS))
    camera.close()
    print('Camera turned off')
    print('Min/Avr/Max fps {}'.format([min(times),sum(times)/len(times),max(times)]))



        
        