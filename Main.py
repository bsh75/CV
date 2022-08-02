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
    device_index = './Data/Snip.avi' # for Boson in USB port
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
    height = getDroneHeight() #m
    #d = int(640*diameter/(height*np.tan(fov)))

    HFOV = 88.3/100*height # From data provided online
    d = int(640*diameter/HFOV) # Ratio equivalence
    return d

def targetPoint(frame, blurKsize):
    """Finds the brightest point in the frame averaged by the a circular kernal with size of spread of water"""
    circ_kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(blurKsize,blurKsize))
    circ_kern = circ_kern/sum(sum(circ_kern))
    frame_CB = cv2.filter2D(frame,-1,circ_kern) # fliter using the circular kernel
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frame_CB)
    return maxLoc

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

def medianFilter(targetLocList):
    """Median filer on the target location list"""
    # Still not the greatest with large buffers
    targetLocListX = [i[0] for i in targetLocList]
    targetLocListY = [i[1] for i in targetLocList]
    targetLocListX.sort()
    targetLocListY.sort()
    targetLoc = (targetLocListX[int(len(targetLocListX)/2)], targetLocListY[int(len(targetLocListY)/2)])
    return targetLoc

def main():
    """Main function which switches between raw and normal feeds"""
    save = False
    rawThresh = 110
    
    width = 640
    height = 512

    N = 3 # Must be a 
    targetLocList = []
    # Read the raw Y16 data from the camera
    cap = init_file()
    if save:
        out = cv2.VideoWriter('RawVid.mp4', -1, 20.0, (640,512))
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        ######## ADD CV PROCESSING BELOW ########
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frameG = frame 
        frame, maxVal, maxLoc = MaxMin(frame)
        
        if maxVal > rawThresh: # Could make this depend on blurring kernal method results (height)
            # Threshold first and black out then find target Loc
            retval, frame = cv2.threshold(frame, rawThresh, 9999, cv2.THRESH_TOZERO)
            blurKsize = getBlurSize()
            targetLoc = targetPoint(frame, blurKsize)
            
            # FIFO buffer with Target Locations
            targetLocList.append(targetLoc)
            if len(targetLocList) > N:
                targetLocList.pop(0)
            
            # Median filtering
            targetLoc = medianFilter(targetLocList)

            print(targetLoc)
            cv2.circle(frame, targetLoc, 3, (255, 255, 255), 3)
            cv2.circle(frame, targetLoc, blurKsize, (255, 255, 255), 2)

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
        cv2.imshow('Norm', frame)
        
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
