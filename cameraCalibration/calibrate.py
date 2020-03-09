#Based on open3d examples/Python/ReconstructionSystem/sensors/realsense_recorder.py

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum
from CharucoBoardToSVG.arucoUtililties import toDict
from cv2 import aruco
import math
import glob 

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

def calibrate(image_paths):
    if len(image_paths) == 0:
        return None
    else:
        charuco_board_params={'squaresX': 6, 'squaresY': 10, 'squareLength': 0.025, 'markerLength':0.02, 'dictionary': 'DICT_4X4_50'}
        
        DICTIONARY = toDict(charuco_board_params['dictionary'])

        # The calibration part is a modified version of https://github.com/kyle-elsalhi/opencv-examples/blob/master/CalibrationByCharucoBoard/CalibrateCamera.py
        # Create constants to be passed into OpenCV and Aruco methods
        CHARUCO_BOARD = aruco.CharucoBoard_create(
            squaresX=charuco_board_params['squaresX'],
            squaresY=charuco_board_params['squaresY'],
            squareLength=charuco_board_params['squareLength'],
            markerLength=charuco_board_params['markerLength'],
            dictionary=toDict(charuco_board_params['dictionary']))# toDict converts string to dict integer

        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        corners_all = []  # Corners discovered in all images processed
        ids_all = []  # Aruco ids corresponding to corners discovered
        image_size = None  # Determined at runtime

        # Loop through images glob'ed
        nrImages=len(image_paths)
        for ii,iname in enumerate(image_paths):
            print("Calculating camera parameters: {}%".format(ii/(nrImages-1)),end='\r')  

            # Open the image
            img = cv2.imread(iname)
            # imgscale the image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find aruco markers in the query image
            corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=DICTIONARY)

            # Outline the aruco markers found in our query image
            img = aruco.drawDetectedMarkers(
                image=img,
                corners=corners)

            # Get charuco corners and ids from detected aruco markers
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=CHARUCO_BOARD)

            # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 6 squares
            if response > 6:
                # Add these corners and ids to our calibration arrays
                corners_all.append(charuco_corners)
                ids_all.append(charuco_ids)

                # Draw the Charuco board we've detected to show our calibrator the board was properly detected
                # img = aruco.drawDetectedCornersCharuco(
                #     image=img,
                #     charucoCorners=charuco_corners,
                #     charucoIds=charuco_ids)

                # If our image size is unknown, set it now
                if not image_size:
                    image_size = gray.shape[::-1]

                # Reproportion the image, maxing width or height at 1000
                proportion = max(img.shape) / 1000.0
                img = cv2.resize(img, (int(img.shape[1] / proportion), int(img.shape[0] / proportion)))
                # cv2.imshow('Charuco board', img)
                # cv2.waitKey(0)

            else:
                print("Not able to detect a charuco board in image: {}".format(iname))


            
        # Destroy any open CV windows

        # Make sure at least one image was found
        if len(image_paths) < 1:
            # Calibration failed because there were no images, warn the user
            print(
                "Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
            # Exit for failure
            exit()

        # Make sure we were able to calibrate on at least one charucoboard by checking
        # if we ever determined the image size
        if not image_size:
            # Calibration failed because we didn't see any charucoboards of the PatternSize used
            print(
                "Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
            # Exit for failure
            exit()

        # Now that we've seen all of our images, perform the camera calibration
        # based on the set of points we've discovered
        calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=corners_all,
            charucoIds=ids_all,
            board=CHARUCO_BOARD,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None)

        # Print matrix and distortion coefficient to the console
    
        print('Camera Matrix: ')
        print(cameraMatrix)
        print('Distance Coefficients: ')
        print(distCoeffs)

if __name__ == "__main__":  
    parser = argparse.ArgumentParser(description="Realsense Recorder. Please select one of the optional arguments")
    parser.add_argument("--output_folder",nargs="?",default='./calibration_data/',help="set output folder")
    # parser.add_argument("charuco_board_json",nargs="?", default='charuco_board.json', help="charuco board json file location")
    args = parser.parse_args()

    calibrate(logImages(args.output_folder))


    
