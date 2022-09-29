"""CV Code for gathering data from tests to later be developed
Allows for switching between video feed modes
Written by Brett Hockey"""

import cv2
import numpy as np

# def init_file(raw):
#     """Load the video file depending on what mode is required"""
#     h = 640
#     w = 512
#     device_index = 1 # for Boson in USB port
#     cap = cv2.VideoCapture(device_index) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
#     if raw:
#         print("get raw")

#         # Load data as Y16 raw
#         cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        
#     return cap

def init_file():
    """Load the video file depending on what mode is required"""
    h = 640
    w = 512
    device_index = 1 #'./Data/Snip.avi' #'RawVid.avi' #./Data/Snip.avi' # for Boson in USB port
    cap = cv2.VideoCapture(device_index)
    #cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    return cap

def init_file_raw():
    """Load the video file"""
    h = 640
    w = 512
    device_index = 1 # for Boson in USB port
    capR = cv2.VideoCapture(device_index, cv2.CAP_DSHOW) # Chnge to 0 instead of filename to get from camera'./Snip.avi'   './Data/MovHotspot.mp4'
    capR.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    capR.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    fourcc = cv2.VideoWriter_fourcc(*'D', 'I', 'V', 'X') # or ('D', 'I', 'V', 'X')Unsure what this does atm but may be needed for Rasberry Pi
    # cv2.VideoWriter.fourcc('Y','1','6',' ')
    capR.set(cv2.CAP_PROP_FOURCC, fourcc)
    capR.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return capR



def map_uint16_to_uint8(img, lower_bound=None, upper_bound=None):
    '''
    Map a 16-bit image trough a lookup table to convert it to 8-bit.

    Parameters
    ----------
    img: numpy.ndarray[np.uint16]
        image that should be mapped
    lower_bound: int, optional
        lower bound of the range that should be mapped to ``[0, 255]``,
        value must be in the range ``[0, 65535]`` and smaller than `upper_bound`
        (defaults to ``numpy.min(img)``)
    upper_bound: int, optional
       upper bound of the range that should be mapped to ``[0, 255]``,
       value must be in the range ``[0, 65535]`` and larger than `lower_bound`
       (defaults to ``numpy.max(img)``)

    Returns
    -------
    numpy.ndarray[uint8]
    '''
    if not(0 <= lower_bound < 2**16) and lower_bound is not None:
        raise ValueError(
            '"lower_bound" must be in the range [0, 65535]')
    if not(0 <= upper_bound < 2**16) and upper_bound is not None:
        raise ValueError(
            '"upper_bound" must be in the range [0, 65535]')
    if lower_bound is None:
        lower_bound = np.min(img)
    if upper_bound is None:
        upper_bound = np.max(img)
    if lower_bound >= upper_bound:
        raise ValueError(
            '"lower_bound" must be smaller than "upper_bound"')
    lut = np.concatenate([
        np.zeros(lower_bound, dtype=np.uint16),
        np.linspace(0, 255, upper_bound - lower_bound).astype(np.uint16),
        np.ones(2**16 - upper_bound, dtype=np.uint16) * 255
    ])
    return lut[img].astype(np.uint8)
# # Let's generate an example image (normally you would load the 16-bit image: cv2.imread(filename, cv2.IMREAD_UNCHANGED))
# img = (np.random.random((100, 100)) * 2**16).astype(np.uint16)

# # Convert it to 8-bit
# map_uint16_to_uint8(img)



def main():
    """Main function which switches between raw and normal feeds"""
    run = True
    raw = False
    save = True
    rawThresh = 110
    print("s: Switches Video Type, q: Quits Program")
    h = 640
    w = 512
    while run:
        # Read the raw Y16 data from the camera
        
        cap = init_file()
        # cap1 = init_file()

        if save:
            fourcc = cv2.VideoWriter_fourcc(*'D', 'I', 'V', 'X') # or ('D', 'I', 'V', 'X')Unsure what this does atm but may be needed for Rasberry Pi

            # fourcc = cv2.VideoWriter_fourcc(*'DIVX') # or ('D', 'I', 'V', 'X')Unsure what this does atm but may be needed for Rasberry Pi
            # fourcc = cv2.VideoWriter.fourcc('Y','1','6',' ') # or ('D', 'I', 'V', 'X')Unsure what this does atm but may be needed for Rasberry Pi
            
            # out = cv2.VideoWriter('media/testOne.mp4', vid_cod, 20.0, (w,h))
            out = cv2.VideoWriter('RawVid.avi', fourcc, 20.0, (w,h))
            

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            w, h = frame.shape[:2]
            # print(w, h)
            ######## ADD CV CODE BELOW ########
            # frame = (frame/256).astype('uint8')
            print(frame[0][0])
            # w, h = frame.shape[:2]
            # print(w, h)
            # frame = map_uint16_to_uint8(frame, 0, 256)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # frame = drawMaxMin(frame)

            ###################################
            # Show the frame in the window
            cv2.imshow('Frame', frame)
            
            if save:
                out.write(frame)
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
        # print(x)
        cap.release()
        out.release()


    cv2.destroyAllWindows()
    
main()
