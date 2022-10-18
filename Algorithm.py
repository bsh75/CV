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
from peripheralFunctions import *
from geoLocationFunctions import *

def singleVid(file, save, DRAW, scatt, litresDisplay, masking, contours, targetInfo, windows, raw):
    """Function processes a single video file: 'file' to display and save depending on
    'save', 'scatt', and 'litres'. The chosen thresholds are also applied"""
    
    # Thresholds
    mildThresh = 100 # Warm Earth
    mediumThresh = 120 # Dimmed Embers
    hotThresh = 230 # Red Hot Embers and Flames
    minDropSize = 5 #L For determining if the blurred area is significant enough to target
    distThresh = 0.5 # Distance threshold for target drop trigger

    maskMultiplier = 4 # Size of mask radius = maskMultiplier*blurKsize

    # Median filter
    N = 5 # Length of median filter buffer
    targetValList = []
    targetLocList = []

    # Run types
    # windows = True # Set to true if running on a windows computer
    # raw = True # If the initialised file is using the raw 16bit data format

    # Initialising flags
    targetAquired = False
    weightedCOMavg = False # Not a flag just needs a default of False


    # Initialise the file
    if (file == 'Camera'):
        cap = init_Camera(raw, windows)
    else:
        cap = init_file(file) # for live operation or video capture: use init_fileCapture(raw, windows)
    width = int(cap.get(3))
    height = int(cap.get(4))
    HFOV = 50 *np.pi/180 # convert 50 deg to radians
    VFOV = HFOV/5*4 # Ratio of pixel width to height
    camSpecs = [width, height, HFOV, VFOV]

    if save:
        # Filename the video will be saved as. String plus the name of the file being processed
        if (file == 'Camera'):
            if raw:
                saveName = 'rawCamera.mp4' #./drawOnCntNum/
            else:
                saveName = 'NormCamera.mp4'
        else:
            saveName = 'AllImg'+file.split('/')[1]+file.split('/')[2] #./drawOnCntNum/
    
        out = cv2.VideoWriter(saveName, -1, 20.0, (width,height))
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        ######## CV PROCESSING BELOW ########
        # Convert to Grayscale and apply the thresholds
        frameG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retval, frameTmild = cv2.threshold(frameG, mildThresh, 9999, cv2.THRESH_TOZERO)
        frameTMild = frameTmild
        if contours:
            retval, frameTmedium = cv2.threshold(frameG, mediumThresh, 9999, cv2.THRESH_TOZERO)
            retval, frameThot = cv2.threshold(frameG, hotThresh, 9999, cv2.THRESH_TOZERO)
        
        # targetFrame is the frame that is blurred to find a target
        targetFrame = frameTmild # Using mild threshold removes information below the threshold
        
        # Gett blur kernel size depending on drones height readings
        blurKsize = getBlurSize(camSpecs)
        
        # Mask size proportional to the size of the blur kernel
        maskRadius = blurKsize*maskMultiplier

        # Keep a lookout for targets that are worth dropping on
        if not targetAquired:
            print('no target')
            # Look for potential targets
            potentialTarget, brightness, frame_CB  = targetPoint(targetFrame, blurKsize)
            # Check if target worth pursueing and adjust flags
            if getLitres(brightness) > minDropSize:
                targetAquired = True
                targetLoc = potentialTarget
                print("target Found")
        
        # If a target has been aquired -> track it
        if targetAquired:
            targetOld = targetLoc
            
            # apply mask around old target to prioritise finding new target near it
            if masking:
                mask = np.zeros_like(targetFrame)
                mask = cv2.circle(mask, targetOld, maskRadius, (255,255,255), -1)
                frameMasked = cv2.bitwise_and(targetFrame, mask, True)
                targetFrame = frameMasked

            # Find new target
            targetLoc, targetVal, frame_CB = targetPoint(targetFrame, blurKsize)    
            print("blurred target")
            targetBlurLoc = targetLoc
            targetBlurVal = targetVal
            
            # Find Contours and use a weighted sum of each contour COM and blurring method to find target
            if contours:
                blurWeight = 20
                COMWeight = 1
                weightedCOMavg, weightedCOMValue, contoursList = contourTargetID(frameTmild, frameTmedium, frameThot, frame_CB)
                # print("loc: ", weightedCOMavg, targetBlurLoc)
                # print("val: ", weightedCOMValue, targetBlurVal)
                if weightedCOMavg:
                    targetLocX = int((blurWeight*targetBlurLoc[0] + COMWeight*weightedCOMavg[0])/(blurWeight + COMWeight))
                    targetLocY = int((blurWeight*targetBlurLoc[1] + COMWeight*weightedCOMavg[1])/(blurWeight + COMWeight))
                    targetLoc = (targetLocX, targetLocY)
                    targetVal = frame_CB[targetLocY, targetLocX]
                    # print("NEW VAL = ", targetVal)
                

            # Median filtering for outlier rejection
            # Still not the greatest with large buffer sizes N (lags behind where it should be)
            # FIFO buffer with Target Locations and Values
            targetLocList.insert(0, targetLoc)
            targetValList.insert(0, targetVal)
            if len(targetLocList) > N:
                targetLocList.pop(-1)
                targetValList.pop(-1)
                targetLoc = medianFilterCoord(targetLocList)
                targetVal = medianFilterVal(targetValList)
            # If the new target found within the mask is not significant enough go back to searching
            if getLitres(targetVal) < minDropSize:
                targetAquired = False
                targetValList = []
                targetLocList = []

            # If target is still confirmed then Geolocate
            if targetAquired:
                targetLocCO = convertToCentreOrigin(targetLoc, camSpecs)
                pixDistance = np.sqrt(targetLocCO[0]**2 + targetLocCO[1]**2) # Just wanting pixel ratios now
                angle = angleFromFront(targetLoc, camSpecs)
                distance = GIMBALsendTarget(pixDistance, angle)
                sendInfoToSA200(distance, angle)
                litres = getLitres(targetVal)
                # If close enough to target then drop the water
                if distance <= distThresh:
                    if getWaterLevel() < litres:
                        litres = 'All'
                    dropWater(litres)

        # Displays
        frameOut = frame #use the unadultered frame: 'frame' to draw on (allows for colour)
        if DRAW:
            if contours:
                if weightedCOMavg:
                    frameOut = cv2.circle(frameOut, weightedCOMavg, 3, (0, 0, 255), 3)
                    frameOut = cv2.circle(frameOut, targetBlurLoc, 3, (0, 255, 0), 3)

            if scatt:
                frameOut = drawScattered(frameOut, frame_CB, camSpecs, litresDisplay)
            

            if targetAquired:
                frameOut = drawCircles(frameOut, targetLoc, blurKsize, maskRadius)
                if targetInfo:
                    frameOut = drawTargetInfo(frameOut, targetLoc, pixDistance, angle, targetVal, litres, camSpecs, litresDisplay)
                if contours:
                    frameOut = drawContours(frameOut, contoursList, thicknessList=[1, 2, 3])
            
            frameOut = drawRefFrame(frameOut, camSpecs)

        ###################################
        if save:
            out.write(frame)
        # Show the frame in the window
        cv2.imshow(str(file), frameOut)
        
        # Close the script if q is pressed.
        # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
    # Release the video file, and close the GUI.
    cap.release()
    if save:
        out.release()


    cv2.destroyAllWindows()
