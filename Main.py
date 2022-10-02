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
from integrationFunctions import *
from geoLocationFunctions import *

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
                

def singleVid(file, save, scattL, litres, mediumThresh, mildThresh, hotThresh, rawThreshAvg, distThresh):
    """Function processes a single video file: 'file' to display and save depending on
    'save', 'scattL', and 'litres'. The chosen thresholds are also applied"""
    cap, device = init_file(file)
    width = int(cap.get(3))
    height = int(cap.get(4))

    i = 0
    if save:
        filename = './drawOnCntNum/'+device.split('/')[1]+device.split('/')[2]
        out = cv2.VideoWriter(filename, -1, 20.0, (640,512))
    
    targetAquired = False
    first = True
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        ######## ADD CV PROCESSING BELOW ########
        # print('drawOnNumCnt'+device.split('/')[1]+device.split('/')[2])
        # Convert to Grayscale and get rid of unessecary information (threshold)
        frameG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retval, frameTmedium = cv2.threshold(frameG, mediumThresh, 9999, cv2.THRESH_TOZERO)
        retval, frameTmild = cv2.threshold(frameG, mildThresh, 9999, cv2.THRESH_TOZERO)
        retval, frameThot = cv2.threshold(frameG, hotThresh, 9999, cv2.THRESH_TOZERO)
        # Gett blur kernel size depending on drones heigh readings
        blurKsize = getBlurSize()
        maskRadius = blurKsize*2

        # Keep a lookout for targets that are worth dropping on
        if not targetAquired:
            print('no target')
            # Look for potential targets
            potentialTarget, brightness, frame_CB  = targetPoint(frameTmedium, blurKsize)
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
            mask = np.zeros_like(frameTmedium)
            mask = cv2.circle(mask, targetOld, maskRadius, (255,255,255), -1)
            frameMasked = cv2.bitwise_and(frameTmedium, mask, True)
            targetLoc, targetVal, frame_CB = targetPoint(frameMasked, blurKsize)
            print("blurred target")
            # Find the distance approximation to the target
            
            # If the new target found within the mask is not significant enough look at rest of frame
            if targetVal < rawThreshAvg:
                print("Reset-----------")
                targetLoc, targetVal, frame_CB = targetPoint(frameTmedium, blurKsize) # Find new target within unmasked frame
                # If the new target is too small then go back to searching algorithm
                if targetVal < rawThreshAvg:
                    targetAquired = False
                    first = False

            # If target is still confirmed then Geolocate
            if targetAquired:
                pixDistance = np.sqrt(targetLoc[0]**2 + targetLoc[1]**2)# distApprox(targetLoc, width, height)  ## Just wanting pixel ratios now
                angle = angleFromFront(targetLoc, width, height)
                distance = GIMBALsendTarget(pixDistance, angle, width, height)
                # If close enough to target then drop the water
                if pixDistance <= distThresh:

                    dropWater(targetVal)

            ## Median filtering
            # Still not the greatest with large buffer sizes N (lags behind where it should be)
            # FIFO buffer with Target Locations
            # targetLocList.append(targetLoc)
            # if len(targetLocList) > N:
            #     targetLocList.pop(0)
            # targetLoc = medianFilter(targetLocList)\

        # litres = getLitres(targetVal)
        # print(frame)
        frameOut = frame

        # frameOut = drawScatteredWeights(frameOut, frame, width, height, scattL, i)
        # i += 1
        # if i == 10:
        #     i = 0
        
        # Find and Draw on Contours ############################### Incorporate into target aquisition
        minArea = 0
        maxNcontours = 3
        areas, contoursMild = contourN(frameTmild, minArea, maxNcontours)
        areas, contoursMedium = contourN(frameTmedium, minArea, maxNcontours)
        areas, contoursHot = contourN(frameThot, minArea, maxNcontours)
        contoursList = [contoursMild, contoursMedium, contoursHot]
        frameOut = drawContours(frameOut, contoursList, thicknessList=[1, 2, 3])

        if targetAquired:
            frameOut = drawCircles(frameOut, targetLoc, blurKsize, maskRadius)
            frameOut = drawTargetInfo(frameOut, targetLoc, pixDistance, angle, targetVal, width, height, litres)

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
