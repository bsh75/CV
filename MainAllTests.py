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
                ['./Main/Raw1.mp4', './Main/Raw2.mp4', './Main/RawVid.mp4', './Main/RawVidHandheldAfter.mp4', './Main/RawVidnHandeldBefore.mp4']]
                

def main():
    """Main function which switches between raw and normal feeds"""
    # Flags
    save = False
    scattL = False
    litres = False # Set to true to display information in terms of drop quantities

    # Thresholds
    mildThresh = 100 # Warm Earth
    mediumThresh = 120 # Dimmed Embers
    hotThresh = 230 # Red Hot Embers and Flames
    rawThreshAvg = 1 # For determining if the blurred area is significant enough to target
    distThresh = 0.5 # Distance threshold for target drop trigger

    # Read the raw Y16 data from the camera
    for files in Y16TestData:
        for file in files:

            singleVid(file, save, scattL, litres, mediumThresh, mildThresh, hotThresh, rawThreshAvg, distThresh)
    
main()