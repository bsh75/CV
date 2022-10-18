"""CV Code for gathering data from tests to later be developed
Allows for switching between video feed modes
Written by Brett Hockey"""

import cv2
import numpy as np
from CVfunctions import init_Camera

def main():
    """Main function which switches between raw and normal feeds
        Is used to save videos to the computer for post-processing"""
    run = True
    raw = True
    save = True
    rawThresh = 110
    windows = False

    while run:
        # Read the raw Y16 data from the camera
        cap = init_Camera(raw, windows)
        if save:
            fourcc = cv2.VideoWriter_fourcc('D', 'I', 'V', 'X') # Unsure what this does atm but may be needed for Rasberry Pi
            if raw:
                out = cv2.VideoWriter('RawVid.mp4', fourcc, 20.0, (640,512))
            else:
                out = cv2.VideoWriter('NormVid.mp4', fourcc, 20.0, (640,512))

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

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