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
    device_index = 1 # for Boson in USB port
    cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    return cap

def MaxMin(frame):
    """Find the maximum and minimum values in the frame and draw them"""
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(frame)
    cv2.circle(frame, maxLoc, 2, (255, 0, 0), 1)
    cv2.putText(frame, str(maxVal), (maxLoc[0]+10, maxLoc[1]+5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.circle(frame, minLoc, 2, (255, 0, 0), 1)
    cv2.putText(frame, str(minVal), (minLoc[0]+10, minLoc[1]+5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    return frame, maxVal, maxLoc

def getDroneHeight():
    """Returns the height of the drone above point in center of frame
    in practice will be aquired from drone onboard computer"""
    height = 74
    return height

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

def main():
    """Main function which switches between raw and normal feeds"""
    save = False
    rawThresh = 110
    target = False

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
        frame, maxVal, maxLoc = MaxMin(frame)
        
        if maxVal > rawThresh: # Could make this depend on blurring kernal method results (height)
            target = True
            ## Start geolocation and sending data to autopilot
            ## Also start classifying the hotspots for drop estimation
            # could be a seperate while loop here instead of 

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
