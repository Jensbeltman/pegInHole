import numpy as np 
import cv2
from ast import literal_eval
from os import makedirs
from os.path import exists, join 
from shutil import rmtree
from CharucoBoardToSVG.arucoUtililties import toDict
import json

# Class for loading JSON values

class ReaderJSON:
    def __init__(self,path):
        file = open(path)
        self.dict = json.load(file)
        file.close()

    def hasKey(self,key):
        return key in self.dict.keys()

    def getNodeVal(self, name, dtype=str):
        return dtype(self.dict[name])

    def getNodeMat(self, name):
        return np.array(self.dict[name])
            
# Reading charuco board json
def readCAUB(path):
    reader = ReaderJSON(path)
    squaresX = reader.getNodeVal("squaresX")                # "14",
    squaresY = reader.getNodeVal("squaresY")                # "20",
    squareLength = reader.getNodeVal("squareLength")      # "0.02",
    markerLength = reader.getNodeVal("markerLength")      # "0.016",
    dictionary = toDict(reader.getNodeVal("dictionary"))        # "DICT_5X5_1000"

    return [squaresX, squaresY, squareLength, markerLength, dictionary]

# Create charuco board object based on path
def createCAUB(path)->cv2.aruco_CharucoBoard:
    squaresX, squaresY, squareLength, markerLength, dictionary = readCAUB(path)
    return cv2.aruco.CharucoBoard_create( squaresX, squaresY, squareLength, markerLength, dictionary )

# Read camera calibration matrix and distortion coefficients from json file
def readCC(path):
    readerCAM = ReaderJSON(path)
    cameraMatrix = readerCAM.getNodeMat('camera_matrix')
    distCoeffs = readerCAM.getNodeMat('distortion_coefficients')
    return cameraMatrix, distCoeffs

# Make a new directory if force is true it will overwrite without prompting
def make_clean_folder(path_folder,force=True):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        if not force:
            user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
            if user_input.lower() == 'y':
                rmtree(path_folder)
                makedirs(path_folder)
            else:
                exit()
        else:
            rmtree(path_folder)
            makedirs(path_folder)
