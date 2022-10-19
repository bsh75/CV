import cv2
from cv2 import blur
import numpy as np
from peripheralFunctions import getDroneHeight
from waterDropRelationship import kdiamModel, params
def init_file(file):
    """Load the video file depending on which mode is required"""
    # deviceIndex = file
    deviceIndex = file # Device Index determines what the code is run on. 0 = Webcam. 1 = USB port on computer
    cap = cv2.VideoCapture(deviceIndex)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    return cap

def init_Camera(raw, windows):
    """Load the video file depending on what mode is required, used for saving videos or  live
        NOTE: cv2.CAP_DSHOW is only neccessary for windows OS"""
    device_index = 1 # for Boson in USB port
    if windows:
        cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW) # REMOVE '+cv2.CAP_DSHOW' for on Rasberry Pi (Linux system)
    else:
        cap = cv2.VideoCapture(device_index) # REMOVE '+cv2.CAP_DSHOW' for on Rasberry Pi (Linux system)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    if raw:
        # Load data as Y16 raw
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' ')) 
    return cap


def getBlurSize(specs, waterLevel):
    """Function uses the drone height and camera specs to specifiy kernel size (also used for )
        NOTE: Either Vertical or Horizontal could be used.
        NOTE: Calibration of DIAMETER is required so it matches area of effect of water,
        this is done dynamically depeneding on the amount of estimated water in tank"""
    # Estimated relationship between water level and area of effect of water
    blurKdiameter = kdiamModel(waterLevel, *params)
    print(blurKdiameter)
    width = specs[0]
    height = specs[1]
    HFOV = specs[2]
    VFOV = specs[3]
    # One method from online data
    h = getDroneHeight() #m
    # Horizontal
    spanX = np.tan(HFOV/2)*h*2
    dH = int(width*blurKdiameter/spanX)
    # Vertical
    spanY = np.tan(VFOV/2)*h*2
    dV = int(height*blurKdiameter/spanY)
    return dH

def getMaskSize(specs, maskDiameter):
    """Function uses the drone height and camera specs to specifiy kernel size (also used for )
        NOTE: Either Vertical or Horizontal could be used.
        NOTE: Calibration of DIAMETER is required so it matches area of effect of water,
        this is done dynamically depeneding on the amount of estimated water in tank"""
    # Estimated relationship between water level and area of effect of water
    width = specs[0]
    height = specs[1]
    HFOV = specs[2]
    VFOV = specs[3]
    # One method from online data
    h = getDroneHeight() #m
    # Horizontal
    spanX = np.tan(HFOV/2)*h*2
    dH = int(width*maskDiameter/spanX)
    # Vertical
    spanY = np.tan(VFOV/2)*h*2
    dV = int(height*maskDiameter/spanY)
    return dH

def targetPoint(frame, blurKsize):
    """Finds the brightest point in the frame averaged by the a circular kernal with size of spread of water"""
    circ_kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(blurKsize,blurKsize))
    circ_kern = circ_kern/sum(sum(circ_kern))
    frameCB = cv2.filter2D(frame,-1,circ_kern) # fliter using the circular kernel
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frameCB)
    # frameBlanked = None # cv2.circle(frame, maxLoc, blurKsize, (0, 0, 0), -1)
    return maxLoc, maxVal, frameCB

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

def contourCOM(contour):
    """Finds the COM"""
    M = cv2.moments(contour)
    COM = None
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        COM = (cx, cy)
    return COM

def contourN(frame, minA, N):
    """Find the N largest contours within the frame"""
    CList = []
    AList = []
    ACList = []
    contours = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
    for c in contours:
        area = cv2.contourArea(c)
        if area > minA:
            ACList.append((area, c))
    if N == 'all':
        N = len(ACList)
    for i in range(0, N):
        if N <= len(ACList):
            max1 = (0, 0)
            for j in range(len(ACList)):
                if ACList[j][0] > max1[0]:
                    max1 = ACList[j]
            ACList.remove(max1)
            CList.append(max1[1])
            AList.append(max1[0])
    return AList, CList

def getLitres(targetVal, waterLevel):
    """Relationship between pixel value and drop quantity"""
    # if targetVal = 250 (approx maximum) then drop whole amount
    L = waterLevel * targetVal / 250
    print(targetVal,"....", L)
    return L


def multiTarget(targetFrame, blurKsize):
    """Find N targets in the frame"""
    N = 3
    i = 0
    targetLoc, targetVal, frame_CB = targetPoint(targetFrame, blurKsize)
    frameBlanked = cv2.circle(targetFrame, targetLoc, blurKsize, (0, 0, 0), -1)
    targetList = []
    targetValList = []
    targetList.append(targetLoc)
    targetValList.append(targetVal)
    while  i < N-1:
        targetLoc, targetVal, frame_CB = targetPoint(frameBlanked, blurKsize)
        targetList.append(targetLoc)
        targetValList.append(targetVal)
        i += 1
    return targetList, targetValList

def contourTargetID(frameMild, frameMedium, frameHot, frameCB):
    """Finds a target and target value using weighted sum of the diffent COMs of
    the largest area contours at each threshold"""
    minArea = 0
    maxNcontours = 3 # set to 'all' to find all contours
    # For each of thresholds, find the top maxNcontours contours
    areasMild, contoursMild = contourN(frameMild, minArea, maxNcontours)
    areasMedium, contoursMedium = contourN(frameMedium, minArea, maxNcontours)
    areasHot, contoursHot = contourN(frameHot, minArea, maxNcontours)
    if len(areasMild) > 0:
        areasMildMaxIndex = areasMild.index(max(areasMild))
        COMmildMax = contourCOM(contoursMild[areasMildMaxIndex])
        mildWeight = 1
    else:
        COMmildMax = (0, 0)
        mildWeight = 0

    if len(areasMedium) > 0:
        areasMediumMaxIndex = areasMedium.index(max(areasMedium))
        COMmediumMax = contourCOM(contoursMedium[areasMediumMaxIndex])
        mediumWeight = 5
    else:
        mediumWeight = 0
        COMmediumMax = (0, 0)

    if len(areasHot) > 0:
        areasHotMaxIndex = areasHot.index(max(areasHot))
        COMhotMax = contourCOM(contoursHot[areasHotMaxIndex])
        hotWeight = 10
    else:
        COMhotMax = (0, 0)
        hotWeight = 0
    
    if (mildWeight == 0) and (mediumWeight == 0) and (hotWeight == 0):
        weightedCOMavg = None
        weightedCOMValue = None
    else:
        weightedCOMaverageX = int((mildWeight*COMmildMax[0] + mediumWeight*COMmediumMax[0] + hotWeight*COMhotMax[0])/(mildWeight + mediumWeight + hotWeight))
        weightedCOMaverageY = int((mildWeight*COMmildMax[1] + mediumWeight*COMmediumMax[1] + hotWeight*COMhotMax[1])/(mildWeight + mediumWeight + hotWeight))
        weightedCOMavg = (weightedCOMaverageX, weightedCOMaverageY)
        weightedCOMValue = frameCB[weightedCOMaverageY, weightedCOMaverageX]
    
    COMsHot = []
    COMsMedium = []
    COMsMild = []
    for cH in contoursHot:
        COMsHot.append(contourCOM(cH))
    for cMe in contoursMedium:
        COMsMedium.append(contourCOM(cMe))
    for cMi in contoursMild:
        COMsMild.append(contourCOM(cMi))
        
    contoursList = [contoursMild, contoursMedium, contoursHot]
    contourCOMsL = [COMhotMax, COMmediumMax, COMmildMax]

    return weightedCOMavg, weightedCOMValue, contoursList, contourCOMsL