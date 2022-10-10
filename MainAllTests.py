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
from Main import singleVid

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
    """Main function which sets all the flags which determine the behaviour of the algorithm.
            -save: If the output of the video is to be saved
            -scatt: If the grid of pixel intensities is to be overlayed
            -litresDisplay: To display the information in terms of litres
    """
    # Flags
    save = False
    scatt = True # Set to True to display a grid of values over the frame
    litresDisplay = True # Set to True to display information in terms of drop quantities
    masking = True # Set to True to use masking when finding the 
    contours = True # Set to True if wanting to use a weighted average of contour COM in Target ID
    targetInfo = True
    # Read the raw Y16 data from the camera
    ALL = True # Set to True to run through all videos consecutively
    DRAW = True
    if ALL:
        for files in Y16TestData:
            for file in files:

                singleVid(file, save, DRAW, scatt, litresDisplay, masking, contours, targetInfo)
    else:
        # Select single file to play using array indexing
        file = Y16TestData[4][0]
        singleVid(file, save, DRAW, scatt, litresDisplay, masking, contours, targetInfo)

main()

"""Make a global CLASS for the camera from which to input all its characteristics"""