import cv2
import numpy as np
from integrationFunctions import getDroneHeight

def init_file(file):
    """Load the video file depending on which mode is required"""
    # deviceIndex = file
    deviceIndex = file # Device Index determines what the code is run on. 0 = Webcam. 1 = USB port on computer
    # deviceIndex = NormalTestData[5][0] # Device Index determines what the code is run on. 0 = Webcam. 1 = USB port on computer
    cap = cv2.VideoCapture(deviceIndex)
    #cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    return cap, deviceIndex

def getBlurSize():
    """Function will later be developed to use the drone height to specifiy kernel size"""
    #fov = 50*np.pi/180 #rad
    diameter = 0.5 #m
    h = getDroneHeight() #m
    #d = int(640*diameter/(height*np.tan(fov)))
    HFOV = 88.28/100*h # From data provided online
    d = int(512*diameter/HFOV) # Ratio equivalence
    return d

def targetPoint(frame, blurKsize):
    """Finds the brightest point in the frame averaged by the a circular kernal with size of spread of water"""
    circ_kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(blurKsize,blurKsize))
    circ_kern = circ_kern/sum(sum(circ_kern))
    frame_CB = cv2.filter2D(frame,-1,circ_kern) # fliter using the circular kernel
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frame_CB)
    return maxLoc, maxVal, frame_CB

def medianFilterCoord(targetLocList):
    """Median filter on the target location list"""
    targetLocListX = [i[0] for i in targetLocList]
    targetLocListY = [i[1] for i in targetLocList]
    targetLocListX.sort()
    targetLocListY.sort()
    targetLoc = (targetLocListX[int(len(targetLocListX)/2)], targetLocListY[int(len(targetLocListY)/2)])
    return targetLoc

def medianFilterVal(targetValList):
    """Median filter on list of Values"""
    targetValList.sort()
    targetVal = targetValList[int(len(targetValList)/2)]
    return targetVal

def contourCOM(frame, contours):
    """Finds the COM"""
    # For contour find the COM and draw it along with the contour itself
    COMs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area > 400:
            frameOut = cv2.drawContours(frame, [c], -1, (255, 255, 0), 2)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                coord = (cx, cy)
                COMs.append(coord)
    # Draw the COM for each contour
    for COM in COMs:
        cv2.circle(frameOut, COM, 5, (255, 255, 0), -1)
    return frameOut

def contourN(frame, minA, N):
    """Find the N largest contours within the frame"""
    C_list = []
    A_list = []
    AC_list = []
    contours = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
    for c in contours:
        area = cv2.contourArea(c)
        if area > minA:
            AC_list.append((area, c))
    for i in range(0, N):
        if N <= len(AC_list):
            max1 = (0, 0)
            for j in range(len(AC_list)):
                if AC_list[j][0] > max1[0]:
                    max1 = AC_list[j]
            AC_list.remove(max1)
            C_list.append(max1[1])
            A_list.append(max1[0])
    return A_list, contours

def getLitres(targetVal):
    """Relationship between pixel value and drop quantity"""
    print(targetVal)
    L = int(1.2 * targetVal - 100)
    if L < 20:
        L = 20
    return L