import numpy as np
from peripheralFunctions import getDroneHeight

def angleFromFront(coordOriginal, width, height):
    """Returns the angle of to the target point with 0degrees being directly infront,
    +ve 0-180 on the right and -ve 0-180 on the left 
    """
    # First the coordinate must be converted to a Center Origin Frame
    coord = convertToCentreOrigin(coordOriginal, width, height)
    # Now find the Angle to coordinate
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

def distApprox(pixDistance, theta, frameWidth, frameHeight):
    """Approximates the distance to the hotspots using flat ground approximation, drone height the camera specifications"""
    # Get drone height for calculations
    h = getDroneHeight()
    # From flir boson camera specs
    HFOV = 88.28/100*h # From data provided online
    VFOV = 77.62/100*h # From data provided online
    # Pixel ratio is the fraction of the screen that the distance takes up in x or y
    pixRatioX = pixDistance*np.sin(theta)/frameWidth
    pixRatioY = pixDistance*np.cos(theta)/frameHeight
    # spanX = 2 * h * np.tan(camAngleX) (= HFOV)
    # spanY = 2 * h * np.tan(camAngleY) (= VFOV)
    dxGuess = pixRatioX * HFOV
    dyGuess = pixRatioY * VFOV
    dGuess = round(np.sqrt(dxGuess**2 + dyGuess**2), 1) # Estimate based on drone height
    return dGuess

def convertToCentreOrigin(originalCoord, width, height):
    """Converts a coordinate in Top Left Origin reference to a Centre Frame Origin referecnce"""
    centerLoc = (int(width/2), int(height/2))
    dxP = originalCoord[0] - centerLoc[0]
    dyP = -(originalCoord[1] - centerLoc[1])
    centreCoord = (dxP, dyP) # Coordinates of target point relative to centre
    return centreCoord
