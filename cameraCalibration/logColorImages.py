import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum

class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5

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

def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)

def logImages(path_output):
    path_color = join(path_output, "color")

    make_clean_folder(path_output)
    make_clean_folder(path_color)

    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # Streaming loop
    frame_count = 0
    image_paths = []
    try:
        cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
        while True:
            key = cv2.waitKey(1)
            
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Get color frame/image
            color_frame = frames.get_color_frame()

            # Validate that both frames are valid
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            
            if key == 32: # space key
                if frame_count == 0:
                    save_intrinsic_as_json(
                        join(path_output, "realsense_camera_intrinsic.json"),
                        color_frame)

                cv2.imwrite("%s/%06d.jpg" % \
                        (path_color, frame_count), color_image)
                image_paths.append("%s/%06d.jpg" % (path_color, frame_count))
                print("Saved color image %06d" % frame_count)
                frame_count += 1

            if key == 27:
                cv2.destroyAllWindows()
                break

            cv2.imshow('Recorder Realsense', color_image)

    finally:
        pipeline.stop()
        return image_paths 

if __name__ == "__main__":  
    parser = argparse.ArgumentParser(description="Realsense Color Image Recorder")
    parser.add_argument("--output_folder",nargs="?",default='./imageLogger',help="set output folder")
    args = parser.parse_args()
    logImages(args.output_folder)