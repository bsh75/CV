"""Computer vision code for thermal camera in wildfire drone applications 
Written by Brett Hockey"""

import cv2
import numpy as np

def init_file(raw):
    """Load the video file depending on what mode is required"""
    device_index = 1 # for Boson in USB port
    cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    if raw:
        # Load data as Y16 raw
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        
    return cap

def drawMaxMin(frame):
    """Find the maximum and minimum values in the frame and draw them"""
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(frame)
    cv2.circle(frame, maxLoc, 2, (255, 0, 0), 1)
    cv2.putText(frame, str(maxVal), (maxLoc[0]+10, maxLoc[1]+5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.circle(frame, minLoc, 2, (255, 0, 0), 1)
    cv2.putText(frame, str(minVal), (minLoc[0]+10, minLoc[1]+5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    return frame

def main():
    """Main function which switches between raw and normal feeds"""
    run = True
    raw = True
    save = True

    while run:
        # Read the raw Y16 data from the camera
        cap = init_file(raw)
        if save:
            if raw:
                out = cv2.VideoWriter('RawVid.mp4', -1, 20.0, (640,512))
            else:
                out = cv2.VideoWriter('NormVid.mp4', -1, 20.0, (640,512))

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            ######## ADD CV CODE BELOW ########
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
            frame = drawMaxMin(frame)

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
            # Switch from Y16 to normal and vice versa when 's' key is pressed
            if cv2.waitKey(10) & 0xFF == ord('s'):
                raw = not raw
                break
        # Release the video file, and close the GUI.
        cap.release()
        out.release()


    cv2.destroyAllWindows()
    
main()
