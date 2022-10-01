"""Computer vision code for thermal camera in wildfire drone applications 
Required behaviour is to be looking for hotsots in frame and when one is 
found start communicating its location to the autopilot to guide to hover 
above. It must also characterise the hotspot to determine how much water 
will be dropped.
Written by Brett Hockey"""
import cv2
import numpy as np

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
                ['./Main/Raw1.mp4', './Main/Raw2.mp4', './Main/RawVid.mp4', './Main/RawVidHandheldAfter', './Main/RawVidnHandeldBefore']]
                

def init_file(file):
    """Load the video file depending on which mode is required"""
    deviceIndex = file
    # deviceIndex = Y16TestData[1][0] # Device Index determines what the code is run on. 0 = Webcam. 1 = USB port on computer
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

def angle(coord):
    """Returns the angle of to the target point with 0degrees being directly infront,
     +ve 0-180 on the right and -ve 0-180 on the left 
     --Target must be in frame with (0, 0) in center """
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

def distApprox(coord, width, height):
    """Approximates the distance to the hotspots using flat ground approximation, drone height the camera specifications"""
    # Get drone height for calculations
    h = getDroneHeight()
    # From flir boson camera specs
    HFOV = 88.28/100*h # From data provided online
    VFOV = 77.62/100*h # From data provided online
    # Pixel ratio is the fraction of the screen that the distance takes up in x or y
    pixRatioX = coord[0]/width
    pixRatioY = coord[1]/height
    # spanX = 2 * h * np.tan(camAngleX) (= HFOV)
    # spanY = 2 * h * np.tan(camAngleY) (= VFOV)
    dxGuess = pixRatioX * HFOV
    dyGuess = pixRatioY * VFOV
    dGuess = round(np.sqrt(dxGuess**2 + dyGuess**2), 1) # Estimate based on drone height
    return dGuess

def getAngle(targetLoc, width, height):
    centerLoc = (int(width/2), int(height/2))
    dxP = targetLoc[0] - centerLoc[0]
    dyP = -(targetLoc[1] - centerLoc[1])
    coord = (dxP, dyP) # Coordinates of target point relative to centre
    theta = angle(coord) # angle that the target point is away from right infront
    return theta

def medianFilter(targetLocList):
    """Median filter on the target location list"""
    targetLocListX = [i[0] for i in targetLocList]
    targetLocListY = [i[1] for i in targetLocList]
    targetLocListX.sort()
    targetLocListY.sort()
    targetLoc = (targetLocListX[int(len(targetLocListX)/2)], targetLocListY[int(len(targetLocListY)/2)])
    return targetLoc

def sendTarget(distance, angle):
    """Send the found target to drones autopilot"""
    successful = 0
    return successful

def dropWater(targetVal):
    """Send the drop command to the valve to release water"""
    amountL = targetVal/100
    return None

def drawRefFrame(frame, width, height):
    """Draws the reference frame on the frame for visual aid"""
    size = 0.7
    thickness = 2
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
    cv2.putText(frameOut, 'Drop: '+str(Litres) + 'L', (targetLoc[0]+Xadj, targetLoc[1]+25),
                                                cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.line(frameOut, centerLoc, targetLoc, colour, thickness)
    frameOut = drawRefFrame(frameOut, width, height)
    return frameOut

def drawScatteredWeights(frameValues, frameDraw, width, height,  i):
    """Display the weightings from a distribution of points in the frame"""
    start = 10
    N = 10
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
            newWeight = getLitres(frameValues[x][y])
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
    L = targetVal - 100
    # Logarithmic??
    return L

def main():
    """Main function which switches between raw and normal feeds"""
    # Flags
    save = False
    targetAquired = False
    first = True

    # Buffer and its size for median filtering (UNUSED)
    targetLocList = [] 
    N = 30
    i = 0

    # Thresholds
    mildThresh = 100 # Warm Earth
    mediumThresh = 120 # Dimmed Embers
    hotThresh = 200 # Red Hot Embers and Flames
    rawThreshAvg = 20 # For determining if the blurred area is significant enough to target
    distThresh = 0.5 # Distance threshold for target drop trigger

    # Read the raw Y16 data from the camera
    for files in Y16TestData:
        for file in files:

            cap, device = init_file(file)

            width = int(cap.get(3))
            height = int(cap.get(4))
            # width = 640
            # height = 512

            if save:
                out = cv2.VideoWriter('./drawOnCnt/'+device.split('/')[1]+device.split('/')[2], -1, 20.0, (640,512))
            
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
                
                ######## ADD CV PROCESSING BELOW ########
                # print('drawOnNumCnt'+device.split('/')[1]+device.split('/')[2])
                # Convert to Grayscale and get rid of unessecary information (threshold)
                frameG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                retval, frameT = cv2.threshold(frameG, mediumThresh, 9999, cv2.THRESH_TOZERO)
                retval, frameTmild = cv2.threshold(frameG, mildThresh, 9999, cv2.THRESH_TOZERO)
                retval, frameThot = cv2.threshold(frameG, hotThresh, 9999, cv2.THRESH_TOZERO)
                # Gett blur kernel size depending on drones heigh readings
                blurKsize = getBlurSize()
                maskRadius = blurKsize*2

                # Keep a lookout for targets that are worth dropping on
                if not targetAquired:
                    print('no target')
                    # Look for potential targets
                    potentialTarget, brightness, frame_CB  = targetPoint(frameT, blurKsize)
                    # Check if target worth pursueing
                    if brightness > rawThreshAvg:
                        targetAquired = True
                        print("target Found")
                
                # If a target has been aquired -> track it within a certain window
                if targetAquired:
                    # Track the target that was first spotted
                    if first:
                        targetLoc = potentialTarget
                        first = False
                    targetOld = targetLoc
                    
                    # apply mask around old target to prioritise finding new target near it
                    mask = np.zeros_like(frameT)
                    mask = cv2.circle(mask, targetOld, maskRadius, (255,255,255), -1)
                    frameMasked = cv2.bitwise_and(frameT, mask, True)
                    targetLoc, targetVal, frame_CB = targetPoint(frameMasked, blurKsize)
                    print("blurred target")
                    # Find the distance approximation to the target
                    
                    # If the new target found within the mask is not significant enough look at rest of frame
                    if targetVal < rawThreshAvg:
                        print("Reset-----------")
                        targetLoc, targetVal, frame_CB = targetPoint(frameT, blurKsize) # Find new target within unmasked frame
                        # If the new target is too small then go back to searching algorithm
                        if targetVal < rawThreshAvg:
                            targetAquired = False
                            first = False

                    # If target is still confirmed then Geolocate
                    if targetAquired:
                        distance = np.sqrt(targetLoc[0]**2 + targetLoc[1]**2)# distApprox(targetLoc, width, height)  ## Just wanting pixel ratios now
                        angle = getAngle(targetLoc, width, height)
                        sendTarget(distance, angle)
                        # If close enough to target then drop the water
                        if distance <= distThresh:

                            dropWater(targetVal)

                    ## Median filtering
                    # Still not the greatest with large buffer sizes N (lags behind where it should be)
                    # FIFO buffer with Target Locations
                    # targetLocList.append(targetLoc)
                    # if len(targetLocList) > N:
                    #     targetLocList.pop(0)
                    # targetLoc = medianFilter(targetLocList)
                    litres = getLitres(targetVal)
                # print(frame)
                frameOut = frameG

                frameOut = drawScatteredWeights(frameOut, frame, width, height, i)
                i += 1
                if i == 10:
                    i = 0
                
                # Find and Draw on Contours ############################### Incorporate into target aquisition
                minArea = 0
                howMany = 3
                colour = (0, 140, 255)
                areas, contours = contourN(frameT, minArea, howMany)
                areas, contoursMild = contourN(frameTmild, minArea, howMany)
                areas, contoursHot = contourN(frameThot, minArea, howMany)
                for i in range(0, len(contoursMild)):
                    cv2.drawContours(frameOut, [contoursMild[i].astype(int)], 0, (0, 140, 255), 1)
                for i in range(0, len(contours)):
                    cv2.drawContours(frameOut, [contours[i].astype(int)], 0, (0, 140, 255), 2)
                for i in range(0, len(contoursHot)):
                    cv2.drawContours(frameOut, [contoursHot[i].astype(int)], 0, (0, 140, 255), 3)
                
                # break
                if targetAquired:
                    thickness = 1
                    colour = (0, 0, 0)
                    cv2.circle(frameOut, targetLoc, 3, colour, thickness)
                    cv2.circle(frameOut, targetLoc, blurKsize, colour, thickness)
                    cv2.circle(frameOut, targetLoc, maskRadius, colour, thickness)
                    # cv2.circle(frameOut, (0, 0), 3, colour, 4)
                    frameOut = draw(frameOut, targetLoc, distance, angle, targetVal, width, height, litres)

                ## Start geolocation and sending data to autopilot
                ## Also start classifying the hotspots for drop estimation
                """Should also include some function which checks whether the target is within a certain 
                distance from targets in the last x amount of frames and if it is then sweet but if it isnt 
                then dont send this position as a coordinate, could also add functionaity such that it 
                blacks out remaining areas around the target Area to focus on one target at a time.
                Essentially need some median/mode filtering remove outliers or Jumps (i think that median will work best)
                Will need to create a buffer of targetLocs that is"""
                # If a new hotspot comes into frame then....
                """Machine learning extension for: -improving drop quantity estimation 
                                                -creating drop sequences for hotspots requiring multiple drops"""

                ###################################
                if save:
                    out.write(frame)
                # Show the frame in the window
                cv2.imshow(str(device), frameOut)
                
                # Close the script if q is pressed.
                # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    run = False
                    break
                
            # Release the video file, and close the GUI.
            cap.release()
            if save:
                out.release()


            cv2.destroyAllWindows()
    
main()