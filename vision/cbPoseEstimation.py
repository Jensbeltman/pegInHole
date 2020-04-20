import numpy as np 
import cv2 
import cv2.aruco as aruco
import glob
import pyrealsense2 as rs
import time
from CharucoBoardToSVG.arucoUtililties import  toDict
import ast
import sys
sys.path.append("../")
from visualServoing.intelRealsenseCamera import CameraRGBD

# Class for loading JSON values
class ReaderJSON(cv2.FileStorage):
    def getNodeVal(self, name, dtype=str):
        node = self.getNode( name)
        return dtype(node.string())

    def getNodeMat(self, name):
        node = self.getNode(name)
        return np.array(ast.literal_eval(node.string()))
            
# load charuco board parameters
arucoParams = cv2.aruco.DetectorParameters_create()

readerCB = ReaderJSON('./json/CB_14_20_20_16_DICT_5X5_1000.json',cv2.FILE_STORAGE_READ)
squaresX = readerCB.getNodeVal("squaresX",int)                # "14",
squaresY = readerCB.getNodeVal("squaresY",int)                # "20",
squareLength = readerCB.getNodeVal("squareLength",float)      # "0.02",
markerLength = readerCB.getNodeVal("markerLength",float)      # "0.016",
dictionary = toDict(readerCB.getNodeVal("dictionary"))        # "DICT_5X5_1000"
readerCB.release()

charucoBoard = cv2.aruco.CharucoBoard_create( squaresX, squaresY, squareLength, markerLength, dictionary )

#load camera calibration
readerCAM = ReaderJSON("./json/camCal.json",cv2.FileStorage_FORMAT_JSON)
cameraMatrix = readerCAM.getNodeMat('cameraMatrix')
distCoeffs = readerCAM.getNodeMat('distCoeffs')
readerCAM.release()

print(cameraMatrix,distCoeffs)

camera = CameraRGBD(init3DVis=False,onlyColor=True,fps=15,path_output='./Data/')

    
while(True):
    # Read frame from Camera
    # convert frame to grayscale
    camera.getFrames()
    frame = camera.color_img
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=arucoParams)  # First, detect markers
    aruco.refineDetectedMarkers(gray, charucoBoard, corners, ids, rejectedImgPoints)

    if ids is not None: # if there is at least one marker detected
        charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, gray, charucoBoard)
        im_with_charuco_board = aruco.drawDetectedCornersCharuco(frame, charucoCorners, charucoIds, (0,255,0))
        rvec, tvec = np.zeros(3),np.zeros(3)
        retval, _, _ = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoBoard, cameraMatrix, distCoeffs,rvec, tvec)  # posture estimation from a charuco board
        if retval == True:
            im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, cameraMatrix, distCoeffs, rvec, tvec, 0.05)  # axis length 100 can be changed according to your requirement
        else:
            im_with_charuco_board = frame
        cv2.imshow("charucoboard", im_with_charuco_board)

    if cv2.waitKey(2) & 0xFF == ord('q'):
        break
 
cv2.destroyAllWindows()
