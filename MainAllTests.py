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

NormalTestData = [['./Data/Small/NormVid.mp4', './Data/Small/NormVidAfter.mp4'],
            ['./Data/Medium/NormVid.mp4', './Data/Medium/NormVidAfter.mp4', './Medium/RawVidAfter.mp4'],
            ['./Data/Large/NormVid.mp4', './Data/Large/NormVidAfter.mp4'],
            ['./Data/Log1/NormVid.mp4', './Data/Log1/NormVidAfter.mp4'],
            ['./Data/Log2/NormVid.mp4', './Data/Log2/NormVidAfter.mp4'],
            ['./Data/Main/Norm1.mp4', './Data/Main/Norm2.mp4', './Data/Main/NormVid.mp4']]

Y16TestData = [['./Data/Small/RawVid.mp4', './Data/Small/RawVidAfter.mp4'],
                ['./Data/Medium/RawVid.mp4', './Data/Medium/RawVidAfter.mp4'],
                ['./Data/Large/RawVid.mp4',  './Data/Large/RawVidAfter.mp4'],
                ['./Data/Log1/RawVid.mp4', './Data/Log1/RawVidAfter.mp4'],
                ['./Data/Log2/RawVid.mp4', './Data/Log2/RawVidAfter.mp4'],
                ['./Data/Main/Raw1.mp4', './Data/Main/Raw2.mp4', './Data/Main/RawVid.mp4', './Data/Main/RawVidHandheldAfter.mp4', './Data/Main/RawVidHandheldBefore.mp4']]
                

def main():
    """Main function which sets all the flags which determine the Mode of Operation and the features used
            - can be set to run through files outlined above or on camera
    """
    # Operation Mode Flags
    videoCaptureF = False # Set to True to run the video capture routine
    save = False # Set to True to generate an output file
    live = False # Set to True if running program live on a camera
    ALL = True # Set to True to run through all videos consecutivelyq
    windows = True # Set to True if running a live feed from windows OS
    raw = False # Set to True if using y16bit data (reccomended)

    # Feature Flags
    masking = True # Set to True to use masking when tracking the target
    contours = True # Set to True if wanting to use a weighted average of contour COM in Target ID
    DRAW = True # Set to True to allow drawing functions
    scatt = True # Set to True to display a grid of values over the frame
    targetInfo = True # Set to True to display target information
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
                file = Y16TestData[5][2]
            CVOperations(file, save, DRAW, scatt, litresDisplay, masking, contours, targetInfo, windows, raw)

main()
