"""CV code containing all the relevant functions that could be used in main
Written by Brett Hockey"""

import cv2
import numpy as np
import time

def init_file_raw():
    """Load the video file"""
    device_index = 1 # for Boson in USB port
    cap = cv2.VideoCapture(device_index, cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  
    return cap

def init_file():
    """Load the video file"""
    cap = cv2.VideoCapture(1)  # Chnge to 0 instead of filename to get from camera
    # get the frame width and height
    
    return cap

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
    return A_list, C_list

def open_close(frame, OC_kernel):
    """Open and close of frame to remove white and black spots"""
    O_frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, OC_kernel)
    OC_frame = cv2.morphologyEx(O_frame, cv2.MORPH_CLOSE, OC_kernel)
    return OC_frame

def getBlurSize():
    """Function will later be developed to use the drone height to specifiy kernel size"""
    blurSize = 6000/getDroneHeight() # 20000 chosen arbritrarily, but will be calibrated during testing
    return blurSize

def targetPoint(frame, blurKsize):
    """Finds the brightest point in the frame averaged by the a circular kernal with size of spread of water"""
    circ_kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(blurKsize,blurKsize))
    circ_kern = circ_kern/sum(sum(circ_kern))
    frame_CB = cv2.filter2D(frame,-1,circ_kern) # fliter using the circular kernel
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frame_CB)
    return maxLoc, maxVal

def getDroneHeight():
    """Returns the height of the drone above point in center of frame
    in practice will be aquired from drone onboard computer"""
    height = 74
    return height

def getAngle(coord):
    """Returns the angle to the target point with 0degrees being directly infront, +ve 0-180 on the right and -ve 0-180 on the left"""
    if coord[1] == 0:
        if coord[0] > 0:
            angle = np.pi/2
        else:
            angle = -np.pi/2
    else:
        angle = np.arctan(coord[0]/coord[1])
    if coord[1] < 0:
        if coord[0] < 0:
            angle -= np.pi
        else:
            angle += np.pi
    return angle

def distApprox(coord, width, height, camAngleX, camAngleY):
    """Approximates the distance to the hotspots using flat ground approximation, drone height the camera specifications"""
    # Pixel ratio is the fraction of the screen that the distance takes up in x or y
    pixRatioX = coord[0]/width
    pixRatioY = coord[1]/height
    # Calculate the approx distance to target point assuming flat ground
    h = getDroneHeight()
    spanX = 2 * h * np.tan(camAngleX)
    spanY = 2 * h * np.tan(camAngleY)
    dxGuess = pixRatioX * spanX
    dyGuess = pixRatioY * spanY
    dGuess = round(np.sqrt(dxGuess**2 + dyGuess**2), 1)
    return dGuess

def drawRefFrame(frame, width, height):
    """Draws the reference frame on the frame for visual aid"""
    topMid = (int(width/2), 0)
    botMid = (int(width/2), height)
    leftMid = (0, int(height/2))
    rightMid = (width, int(height/2))
    cv2.line(frame, topMid, botMid, (255, 0, 0), 2)
    cv2.line(frame, leftMid, rightMid, (255, 0, 0), 2)
    cv2.putText(frame, '0deg', (topMid[0]+5, topMid[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 3)
    cv2.putText(frame, '90deg', (rightMid[0]-90, rightMid[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 3)
    cv2.putText(frame, '-90deg', (leftMid[0], leftMid[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 3)
    return frame

def distanceBetween(targetLoc, width, height, camAngleX, camAngleY):
    """Approximates distance between points"""
    if len(targetLoc) >= 2:
        h = getDroneHeight()
        spanX = 2 * h * np.tan(camAngleX)
        spanY = 2 * h * np.tan(camAngleY)
        dy = spanX*(targetLoc[1][1]-targetLoc[0][1])/width
        dx = spanY*(targetLoc[1][0]-targetLoc[0][0])/height
        d = round(np.sqrt(dx**2 + dy**2), 1)
    return d

def multiTarget(ze_frame, blurKsize, dropWeight, radius):
    maxSum = 0
    targetLoc = []
    targetAreaIntensity = []
    j = 0
    while maxSum <= dropWeight:
        maxLoc, maxVal = targetPoint(ze_frame, blurKsize)
        targetLoc.append(maxLoc)
        targetAreaIntensity.append(maxVal)
        maxSum += maxVal
        cv2.circle(ze_frame, maxLoc, radius, (0, 0, 255), cv2.FILLED)
        j += 1
    return targetLoc, targetAreaIntensity

def draw(frameOut, targetLoc, dGuess, theta, targetAreaIntensity, centerLoc, width, height):
    for j in range(0, len(targetLoc)):
        cv2.circle(frameOut, targetLoc[j], 5, (255, 0, 0), 2)
        cv2.putText(frameOut, str(dGuess[j])+'m '+str(round(theta[j]*180/np.pi, 1))+'deg', (targetLoc[j][0]+30, targetLoc[j][1]+0),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(frameOut, 'Weighting: '+str(targetAreaIntensity[j]), (targetLoc[j][0]+30, targetLoc[j][1]+15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.line(frameOut, centerLoc, targetLoc[j], (255, 0, 0), 2)
    frameOut = drawRefFrame(frameOut, width, height)

def geolocate(targetLoc, centerLoc, camAngleX, camAngleY, width, height):
    dxP = []
    dyP = []
    coord = []
    theta = []
    dGuess =[]

    for k in range(0, len(targetLoc)):
        dxP.append(targetLoc[k][0] - centerLoc[0])
        dyP.append(-(targetLoc[k][1] - centerLoc[1]))
        coord.append((dxP[k], dyP[k])) # Coordinates of target point relative to centre
        dGuess.append(distApprox(coord[k], width, height, camAngleX, camAngleY))
        # Get angle and approx distance
        theta.append(getAngle(coord[k])) # angle that the target point is away from right infront
    return dGuess, theta

def tempEst(targetLoc, frame):
    T = []
    for i in range(0, len(targetLoc)):
        K = 0.01 * frame[targetLoc[i][1], targetLoc[i][0]]
        T.append(K - 273.15)
    return T

def drawMaxMin(frame):
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(frame)
    cv2.circle(frame, maxLoc, 5, (255, 0, 0), 2)
    cv2.putText(frame, str(maxVal), (maxLoc[0]+30, maxLoc[1]+15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.circle(frame, minLoc, 5, (255, 0, 0), 2)
    cv2.putText(frame, str(minVal), (minLoc[0]+30, minLoc[1]+15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    return frame

def main():
    # Open file and get width and height of frame
    cap = init_file_raw()
    #cap = init_file()

    width = int(cap.get(3))
    height = int(cap.get(4))
    centerLoc = (int(width/2), int(height/2))
    # Camera angles from specification converted to radian
    camAngleX = 43/2 * np.pi/180 
    camAngleY = 50/2 * np.pi/180 

    # Selection for threshold values
    threshold1 = 100 ### Exact value will be determined during later testing (MAY NEED DYNAMIC CALIBRATION)
    minA = 200 ### Nessecary? (WIll need dynamic calibration)
    N = 5 # Largest N contours will be used

    # Open Close Kernel
    OC_kernel = np.ones((10, 10), np.int8)

    # Relates to capacity of the drop
    dropWeight = 350 ### Will need  calibration

    while cap.isOpened():

        ret, frame = cap.read()
        if not ret:
            break
        
        # Thresholding to zero to set cooler areas to 0 and keep the rest
        retval, ze_frame = cv2.threshold(frame, threshold1, 9999, cv2.THRESH_TOZERO)

        # Blur kernel determination
        blurKsize = int(getBlurSize()) ### make this function
        radius = int(blurKsize/2)

        # Find the brightest n targets until estimated WATERRUNOUT
        targetLoc, targetAreaIntensity = multiTarget(ze_frame, blurKsize, dropWeight, radius)
        frameOut = frame
        T = tempEst(targetLoc, frame)

        frame = drawMaxMin(frame)

        # Find angles and estimate distances between center and targets
        dGuess, theta = geolocate(targetLoc, centerLoc, camAngleX, camAngleY, width, height)

        # Draw information on the frame
        #draw(frameOut, targetLoc, dGuess, theta, T, centerLoc, width, height)

        # Show the frame in the window with threshold trackbars
        cv2.imshow('Y16', frameOut) 
        #ret, frame1 = cap1.read()
        # frame1 = frame
        #cv2.imshow('Norm', frame1) 
        
        # Close the script if q is pressed.
        # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    # Release the video file, and close the GUI.
    cap.release()
    cv2.destroyAllWindows()
    
main()
