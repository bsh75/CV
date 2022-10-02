import cv2
import numpy as np

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

def getDroneHeight():
    """Returns the height of the drone above point in center of frame in practice 
    this will be the function that recieves height from drone onboard computer"""
    height = 2.2 #m
    return height

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

def angleFromFront(coordOriginal, width, height):
    """Returns the angle of to the target point with 0degrees being directly infront,
    +ve 0-180 on the right and -ve 0-180 on the left 
    """
    # First the coordinate must be converted to a Center Origin Frame
    coord = convertToCentreOrigin(coordOriginal, width, height)
    # Now find the Angle to coordinate
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

def distApprox(pixDistance, theta, frameWidth, frameHeight):
    """Approximates the distance to the hotspots using flat ground approximation, drone height the camera specifications"""
    # Get drone height for calculations
    h = getDroneHeight()
    # From flir boson camera specs
    HFOV = 88.28/100*h # From data provided online
    VFOV = 77.62/100*h # From data provided online
    # Pixel ratio is the fraction of the screen that the distance takes up in x or y
    pixRatioX = pixDistance*np.sin(theta)/frameWidth
    pixRatioY = pixDistance*np.cos(theta)/frameHeight
    # spanX = 2 * h * np.tan(camAngleX) (= HFOV)
    # spanY = 2 * h * np.tan(camAngleY) (= VFOV)
    dxGuess = pixRatioX * HFOV
    dyGuess = pixRatioY * VFOV
    dd = np.sqrt(dxGuess**2 + dyGuess**2)
    print(theta)
    print(np.sin(theta))
    print(type(dd))
    dGuess = round(np.sqrt(dxGuess**2 + dyGuess**2), 1) # Estimate based on drone height
    return dGuess

def convertToCentreOrigin(originalCoord, width, height):
    """Converts a coordinate in Top Left Origin reference to a Centre Frame Origin referecnce"""
    centerLoc = (int(width/2), int(height/2))
    dxP = originalCoord[0] - centerLoc[0]
    dyP = -(originalCoord[1] - centerLoc[1])
    centreCoord = (dxP, dyP) # Coordinates of target point relative to centre
    return centreCoord

def medianFilter(targetLocList):
    """Median filter on the target location list"""
    targetLocListX = [i[0] for i in targetLocList]
    targetLocListY = [i[1] for i in targetLocList]
    targetLocListX.sort()
    targetLocListY.sort()
    targetLoc = (targetLocListX[int(len(targetLocListX)/2)], targetLocListY[int(len(targetLocListY)/2)])
    return targetLoc

def GIMBALsendTarget(pixDistance, angle, frameWidth, frameHeight):
    """Send the bearing and heading to GIMBAL and return an actual distance calculation"""
    # For now just use trigonometric flat ground approximation
    distance = distApprox(pixDistance, angle, frameWidth, frameHeight)
    return distance

def dropWater(targetVal):
    """Send the drop command to the valve to release water"""
    amountL = targetVal/100
    print("DROPPING - DROPPING - DROPPING - DROPPING - DROPPING - DROPPING")
    return None

def drawRefFrame(frame, width, height):
    """Draws the reference frame on the frame for visual aid"""
    size = 0.7
    thickness = 1
    colour = (0, 0, 0)
    gap = 40
    topMid = (int(width/2), 0)
    botMid = (int(width/2), height)
    leftMid = (0, int(height/2))
    rightMid = (width, int(height/2))
    cv2.line(frame, (topMid[0], topMid[1]+gap), (botMid[0], botMid[1]-gap), colour, thickness)
    cv2.line(frame, (leftMid[0]+gap+10, leftMid[1]), (rightMid[0]-gap-10, rightMid[1]), colour, thickness)
    cv2.putText(frame, '-0+', (topMid[0]-25, topMid[1]+30), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.putText(frame, '+90', (rightMid[0]-50, rightMid[1]+5), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.putText(frame, '-90', (leftMid[0], leftMid[1]+5), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.putText(frame, '-180+', (botMid[0]-40, botMid[1]-10), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    return frame

def draw(frameOut, targetLoc, dGuess, theta, weighting, width, height, Litres):
    size = 0.5
    thickness = 2
    colour = (0, 0, 0)
    Xadj = 20
    centerLoc = (int(width/2), int(height/2))
    weighting = int(weighting)
    cv2.circle(frameOut, targetLoc, 5, colour, thickness) 
    cv2.putText(frameOut, str(int(dGuess))+'pix '+str(round(theta*180/np.pi, 1))+'deg', (targetLoc[0]+Xadj, targetLoc[1]+0), 
                                                cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    if Litres:
        cv2.putText(frameOut, 'Drop: '+str(Litres) + 'L', (targetLoc[0]+Xadj, targetLoc[1]+25),
                                                cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    else:
        cv2.putText(frameOut, 'Weight: '+str(weighting), (targetLoc[0]+Xadj, targetLoc[1]+25),
                                                cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.line(frameOut, centerLoc, targetLoc, colour, thickness)
    # frameOut = drawRefFrame(frameOut, width, height)
    return frameOut

def drawScatteredWeights(frameValues, frameDraw, width, height, scattL, i):
    """Display the weightings from a distribution of points in the frame"""
    start = 10
    N = 20
    Y = np.linspace(0, width, N, dtype=int)
    X = np.linspace(0, height, N, dtype=int)
    points = []
    size = 0.4
    thickness = 1
    colour = (0, 140, 255)
    cv2.putText(frameDraw, str(frameValues[X[-2]][Y[-2]]), (X[-1], Y[-4]), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    for x in X[1:-1]:
        row = []
        for y in Y[1:-1]:
            newPoint = (y, x)
            if scattL:
                newWeight = getLitres(frameValues[x][y])
            else:
                newWeight = frameValues[x][y]
            # if not i:
            #     point = newPoint
            #     weight = newWeight
            row.append(newPoint)
            cv2.putText(frameDraw, str(newWeight), newPoint, cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
        points.append(row)
        
    return frameDraw

def contourCOM(bin_frame, frame):
    """Contour and contour COM finding funciton which also draws results onto frame"""
    # Find contours for both thresholds
    contours = cv2.findContours(bin_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
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
    L = int(1.2 * targetVal - 100)
    if L < 20:
        L = 20
    return L