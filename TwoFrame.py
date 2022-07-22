"""Computer vision code for thermal camera in wildfire drone applications 
Written by Brett Hockey"""

import cv2
import numpy as np

def init_file_raw():
    """Load the video file"""
    device_index = 1 # for Boson in USB port
    capR = cv2.VideoCapture(device_index, cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    capR.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    capR.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capR.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    capR.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return capR

def init_file():
    cap = cv2.VideoCapture(1)
    return cap

def main():
    # Open file and get width and height of frame
    capR = init_file_raw()

    while capR.isOpened():

        ret, frame = capR.read()

        if not ret:
            break
        
        # Show the frame in the window with threshold trackbars
        cv2.imshow('Norm', frame)
        
        # Close the script if q is pressed.
        # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    # Release the video file, and close the GUI.
    capR.release()


    cap = init_file()
    
    while cap.isOpened():

        ret, frame = cap.read()

        if not ret:
            break
        
        # Show the frame in the window with threshold trackbars
        cv2.imshow('Norm', frame)
        
        # Close the script if q is pressed.
        # Note that the delay in cv2.waitKey affects how quickly the video will play on screen.
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


    cv2.destroyAllWindows()
    
main()
