import numpy as np 
import cv2 
import cv2.aruco as aruco
import glob
import pyrealsense2 as rs
import time
import sys
from utility import *


if __name__ == "__main__":
    from intelRealsenseCamera import *

    # Create charuco board and camera calibration variables from file
    path_camCal = './json/cam_cal_D415_1280x720.json'
    path_CAUB = './json/CB_14_20_20_16_DICT_5X5_1000.json'
    squaresX, squaresY, squareLength, markerLength, dictionary = readCAUB(path_CAUB)
    charucoBoard = cv2.aruco.CharucoBoard_create(squaresX, squaresY, squareLength, markerLength, dictionary)
    cameraMatrix,distCoeffs = readCC(path_camCal)
    arucoParams = cv2.aruco.DetectorParameters_create()
    print(cameraMatrix,distCoeffs)

    # Intel realsense camera with only color = True 
    camera = CameraRGBD(path_camCal = path_camCal ,fps=15, path_camera_preset=PresetPaths.custom,path_output='./data/RGBDData/',visualization=False)
    while not camera.shutdown_flag.is_set():
        # Read frame from Camera
        camera.getFrames()

        
        cv2.imshow("image", camera.color_img)

        # convert frame to grayscale
        gray = cv2.cvtColor(camera.color_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=arucoParams)  # First, detect markers
        aruco.refineDetectedMarkers(gray, charucoBoard, corners, ids, rejectedImgPoints)

        if ids is not None: # if there is at least one marker detected
            charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, gray, charucoBoard)
            im_with_charuco_board = aruco.drawDetectedCornersCharuco(gray, charucoCorners, charucoIds, (0,255,0))
            rvec, tvec = np.zeros(3),np.zeros(3)
            retval, _, _ = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoBoard, cameraMatrix, distCoeffs,rvec, tvec)  # posture estimation from a charuco board
            if retval == True:
                im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, cameraMatrix, distCoeffs, rvec, tvec, 0.05)  # axis length 100 can be changed according to your requirement
            


    camera.close()
    cv2.destroyAllWindows()
