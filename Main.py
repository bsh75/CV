"""Computer vision code for thermal camera in wildfire drone applications 
Required behaviour is to be looking for hotsots in frame and when one is 
found start communicating its location to the autopilot to guide to hover 
above. It must also characterise the hotspot to determine how much water 
will be dropped.

Written by Brett Hockey"""

import cv2
import numpy as np

def init_file():
    """Load the video file depending on what mode is required"""
    device_index = 1 #'./Data/Snip.avi' #'RawVid.avi' #./Data/Snip.avi' # for Boson in USB port
    cap = cv2.VideoCapture(device_index)
    #cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    return cap

def MaxMin(frame):
    """Find the maximum and minimum values in the frame and draw them"""
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(frame)
    '''cv2.circle(frame, maxLoc, 2, (255, 0, 0), 1)
    cv2.putText(frame, str(maxVal), (maxLoc[0]+10, maxLoc[1]+5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.circle(frame, minLoc, 2, (255, 0, 0), 1)
    cv2.putText(frame, str(minVal), (minLoc[0]+10, minLoc[1]+5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)'''
    return frame, maxVal, maxLoc

def getDroneHeight():
    """Returns the height of the drone above point in center of frame
    in practice will be aquired from drone onboard computer"""
    height = 8
    return height

def getBlurSize():
    """Function will later be developed to use the drone height to specifiy kernel size"""
    #fov = 50*np.pi/180 #rad
    diameter = 0.5 #m
    h = getDroneHeight() #m
    #d = int(640*diameter/(height*np.tan(fov)))
    HFOV = 88.28/100*h # From data provided online
    d = int(640*diameter/HFOV) # Ratio equivalence
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

def draw(frameOut, targetLoc, dGuess, theta, targetAreaIntensity, width, height):
    Xadj = 50
    size = 0.7
    centerLoc = (int(width/2), int(height/2))
    cv2.circle(frameOut, targetLoc, 5, (255, 0, 0), 2)
    cv2.putText(frameOut, str(int(dGuess))+'pix '+str(round(theta*180/np.pi, 1))+'deg', (targetLoc[0]+Xadj, targetLoc[1]+0),cv2.FONT_HERSHEY_SIMPLEX, size, (255, 0, 0), 2)
    cv2.putText(frameOut, 'Weighting: '+str(targetAreaIntensity), (targetLoc[0]+Xadj, targetLoc[1]+25),cv2.FONT_HERSHEY_SIMPLEX, size, (255, 0, 0), 2)
    cv2.line(frameOut, centerLoc, targetLoc, (255, 0, 0), 2)
    frameOut = drawRefFrame(frameOut, width, height)
    return frameOut

def main():
    """Main function which switches between raw and normal feeds"""
    # Flags
    save = False
    targetAquired = False
    first = True

    # Buffer and its size for median filtering (UNUSED)
    targetLocList = [] 
    N = 30

    # Thresholds
    rawThresh = 100 # For thresholding (removing unneccessary information)
    rawThreshAvg = 20 # For determining if the blurred area is significant enough to target
    distThresh = 0.5 # Distance threshold for target drop trigger

    # Read the raw Y16 data from the camera
    cap = init_file()

    width = int(cap.get(3))
    height = int(cap.get(4))
    # width = 640
    # height = 512

    if save:
        out = cv2.VideoWriter('RawVid.mp4', -1, 20.0, (640,512))
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        ######## ADD CV PROCESSING BELOW ########

        # Convert to Grayscale and get rid of unessecary information (threshold)
        frameG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # frameT = frameG
        retval, frameT = cv2.threshold(frameG, rawThresh, 9999, cv2.THRESH_TOZERO)
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

        frameOut = frame_CB
        if targetAquired:
            cv2.circle(frameOut, targetLoc, 3, (255, 255, 255), 3)
            cv2.circle(frameOut, targetLoc, blurKsize, (255, 255, 255), 2)
            cv2.circle(frameOut, targetLoc, maskRadius, (255, 255, 255), 1)
        cv2.circle(frameOut, (0, 0), 3, (255, 255, 255), 4)
        frameOut = draw(frameOut, targetLoc, distance, angle, targetVal, width, height)

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
        cv2.imshow('Norm', frameOut)
        
        # Close the script if q is pressed.
        # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
        if cv2.waitKey(10) & 0xFF == ord('q'):
            run = False
            break
        
    # Release the video file, and close the GUI.
    cap.release()
    out.release()


    cv2.destroyAllWindows()
    
main()
