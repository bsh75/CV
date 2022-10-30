import numpy as np
from peripheralFunctions import getDroneHeight

def angleFromFront(coordOriginal, specs):
    """Returns the angle of to the target point with 0degrees being directly infront,
    +ve 0-180 on the right and -ve 0-180 on the left 
    """
    width = specs[0]
    height = specs[1]
    # First the coordinate must be converted to a Center Origin Frame
    coord = convertToCentreOrigin(coordOriginal, specs)
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

def distApprox(pixDistance, theta, specs):
    """Approximates the distance to the hotspots using flat ground approximation, drone height the camera specifications"""
    width = specs[0]
    height = specs[1]
    HFOV = specs[2]
    VFOV = specs[3]
    # Get drone height for calculations
    h = getDroneHeight()
    # Pixel ratio is the fraction of the screen that the distance takes up in x or y
    pixRatioX = pixDistance*np.sin(theta)/width
    pixRatioY = pixDistance*np.cos(theta)/height
    spanX = 2 * h * np.tan(HFOV)
    spanY = 2 * h * np.tan(VFOV)
    dxGuess = pixRatioX * spanX
    dyGuess = pixRatioY * spanY
    dGuess = round(np.sqrt(dxGuess**2 + dyGuess**2), 1) # Estimate based on drone height
    return dGuess

def convertToCentreOrigin(originalCoord, specs):
    """Converts a coordinate in Top Left Origin reference to a Centre Frame Origin referecnce"""
    width = specs[0]
    height = specs[1]
    centerLoc = (int(width/2), int(height/2))
    dxP = originalCoord[0] - centerLoc[0]
    dyP = -(originalCoord[1] - centerLoc[1])
    centreCoord = (dxP, dyP) # Coordinates of target point relative to centre
    return centreCoord
