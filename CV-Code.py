"""Computer vision code for thermal camera in wildfire drone applications 
Written by Brett Hockey"""

import cv2
import numpy as np
import time

def init_file():
    """Load the video file"""
    cap = cv2.VideoCapture('./Data/Snip.avi')  # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    return cap

def contourN(frame, minA, N):
    """Find the N largest contours qithin the frame"""
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
    h = getDroneHeight()
    spanX = 2 * h * np.tan(camAngleX)
    spanY = 2 * h * np.tan(camAngleY)
    dy = spanX*(targetLoc[1][1]-targetLoc[0][1])/width
    dx = spanY*(targetLoc[1][0]-targetLoc[0][0])/height
    d = round(np.sqrt(dx**2 + dy**2), 1)
    return d


def main():
    # Open file and get width and height of frame
    cap = init_file()
    width = int(cap.get(3))
    height = int(cap.get(4))
    centerLoc = (int(width/2), int(height/2))
    # Camera angles from specification converted to radian
    camAngleX = 43/2 * np.pi/180 
    camAngleY = 50/2 * np.pi/180 

    # Selection for threshold values
    threshold1 = 100 # Exact value will be determined during later testing
    minA = 200 # Minimum size of any contour
    N = 5 # Largest N contours will be used

    # detector = blob_init()
    OC_kernel = np.ones((10, 10), np.int8)
    
    elapsed_time = []

    dropWeight = 350
    while cap.isOpened():
        # Read frames from file until no more
        start_time = time.time()
        ret, frame = cap.read()
        #frame = cv2.resize(frame, None, None, fx=0.5, fy=0.5)
        if not ret:
            break
        
        # Grey & Blur
        frame = cv2.GaussianBlur(frame, (5, 5), 0) # Apply gaussian filtering for blur
        frameG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the frame to grayscale.
        
        # Thresholding to zero to set cooler areas to 0 and keep the rest
        retval, ze_frame = cv2.threshold(frameG, threshold1, 9999, cv2.THRESH_TOZERO)
        # Binary Thresholds
        retval, bin_frame = cv2.threshold(frameG, threshold1, 9999, cv2.THRESH_BINARY)

        # Opening/Closing to remove small white/black dots in frame to save computation on Contours
        OC_frame = open_close(bin_frame, OC_kernel)

        # Find the N maximum contours
        areas, maxContours = contourN(OC_frame, minA, N)

        # Blur kernel determination
        blurKsize = int(getBlurSize())
        radius = int(blurKsize/2)

        # Check whether the size of the summed target point brightest values
        maxSum = 0
        targetLoc = []
        targetAreaIntensity = []
        j = 0
        while maxSum <= dropWeight and j < len(maxContours):
            maxLoc, maxVal = targetPoint(ze_frame, blurKsize)
            
            if cv2.pointPolygonTest(maxContours[j], maxLoc, True):
                targetLoc.append(maxLoc)
                targetAreaIntensity.append(maxVal)
                maxSum += maxVal
                cv2.circle(ze_frame, maxLoc, radius, (0, 0, 255), cv2.FILLED)
            j += 1
        
        frameOut = ze_frame
        
        for j in range(0, len(targetLoc)):
            cv2.circle(frameOut, targetLoc[j], 5, (255, 0, 0), 2)
        
        if len(targetLoc) >= 2:
            d = distanceBetween(targetLoc, width, height, camAngleX, camAngleY)
            print(d)

        
        # Find distance between center and target in terms of pixels
        dxP = []
        dyP = []
        coord = []
        theta = []

        for k in range(0, len(targetLoc)):
            dxP.append(targetLoc[k][0] - centerLoc[0])
            dyP.append(-(targetLoc[k][1] - centerLoc[1]))
            coord.append((dxP[k], dyP[k])) # Coordinates of target point relative to centre
            dGuess = distApprox(coord[k], width, height, camAngleX, camAngleY)
            # Get angle and approx distance
            theta.append(getAngle(coord[k])) # angle that the target point is away from right infront
            cv2.putText(frameOut, str(dGuess)+'m '+str(round(theta[k]*180/np.pi, 1))+'deg', (targetLoc[k][0]+30, targetLoc[k][1]+0),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(frameOut, 'Weighting: '+str(targetAreaIntensity[k]), (targetLoc[k][0]+30, targetLoc[k][1]+15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.line(frameOut, centerLoc, targetLoc[k], (255, 0, 0), 2)
        
        frameOut = drawRefFrame(frameOut, width, height)     
        
        # Show the frame in the window with threshold trackbars
        cv2.imshow('Threshold to Zero', frameOut) 
        current_time = time.time()
        elapsed_time.append(current_time - start_time)
        
        # Close the script if q is pressed.
        # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    print(sum(elapsed_time)/len(elapsed_time))        
    # Release the video file, and close the GUI.
    cap.release()
    cv2.destroyAllWindows()
    
main()
