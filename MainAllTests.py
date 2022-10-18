"""Computer vision code for thermal camera in wildfire drone applications 
Required behaviour is to be looking for hotsots in frame and when one is 
found start communicating its location to the autopilot to guide to hover 
above. It must also characterise the hotspot to determine how much water 
will be dropped.
Written by Brett Hockey"""
import cv2
import numpy as np
from CVfunctions import *
from drawFunctions import *
from Algorithm import CVOperations
from VideoCapture import vidCapture

NormalTestData = [['./Small/NormVid.mp4', './Small/NormVidAfter.mp4'],
            ['./Medium/NormVid.mp4', './Medium/NormVidAfter.mp4', './Medium/RawVidAfter.mp4'],
            ['./Large/NormVid.mp4', './Large/NormVidAfter.mp4'],
            ['./Log1/NormVid.mp4', './Log1/NormVidAfter.mp4'],
            ['./Log2/NormVid.mp4', './Log2/NormVidAfter.mp4'],
            ['./Main/Norm1.mp4', './Main/Norm2.mp4', './Main/NormVid.mp4']]

Y16TestData = [['./Small/RawVid.mp4', './Small/RawVidAfter.mp4'],
                ['./Medium/RawVid.mp4', './Medium/RawVidAfter.mp4'],
                ['./Large/RawVid.mp4',  './Large/RawVidAfter.mp4'],
                ['./Log1/RawVid.mp4', './Log1/RawVidAfter.mp4'],
                ['./Log2/RawVid.mp4', './Log2/RawVidAfter.mp4'],
                ['./Main/Raw1.mp4', './Main/Raw2.mp4', './Main/RawVid.mp4', './Main/RawVidHandheldAfter.mp4', './Main/RawVidHandheldBefore.mp4']]
                

def main():
    """Main function which sets all the flags which determine the Mode of Operation and the features used
            - can be set to run through files outlined above or on camera
    """
    # Operation Mode Flags
    videoCaptureF = False # Set to True to run the video capture routine
    save = False # Set to True to generate an output file
    live = False # Set to True if running program live on a camera
    ALL = False # Set to True to run through all videos consecutively
    windows = True # Set to True if running a live feed from windows OS
    raw = False # Set to True if using y16bit data (reccomended)

    # Feature Flags
    masking = False # Set to True to use masking when tracking the target
    contours = False # Set to True if wanting to use a weighted average of contour COM in Target ID
    DRAW = True # Set to True to allow drawing functions
    scatt = False # Set to True to display a grid of values over the frame
    targetInfo = False # Set to True to display target information
    litresDisplay = True # Set to True to display scatt values and targetInfo in terms of drop quantities (using getLitres())

    # If just wanting to capture and record video for later analysis
    if videoCaptureF:
        vidCapture(save, windows, raw)
    else:
        if ALL:
            for files in Y16TestData:
                for file in files:

                    CVOperations(file, save, DRAW, scatt, litresDisplay, masking, contours, targetInfo, windows, raw)
        else:
            if live:
                file = 'Camera'
            else:
                # Select single file to play using array indexing
                file = Y16TestData[4][0]
            CVOperations(file, save, DRAW, scatt, litresDisplay, masking, contours, targetInfo, windows, raw)

main()
