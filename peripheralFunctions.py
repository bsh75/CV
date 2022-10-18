import numpy as np
# from geoLocationFunctions import distApprox


def getDroneHeight():
    """Returns the height of the drone above point in center of frame in practice 
    this will be the function that recieves height from drone onboard computer"""
    height = 2.4 #m
    return height

def GIMBALsendTarget(pixDistance, angle):
    """Send the bearing and heading to GIMBAL and return an actual distance calculation"""
    # For now just use trigonometric flat ground approximation
    distance = 5# distApprox(pixDistance, angle, frameWidth, frameHeight)
    return distance

def dropWater(amountL):
    """Send the drop command to the valve to release water"""
    if amountL == 'All':
        print("DROPPING ALL - DROPPING ALL")
        # And then return home
    else:
        print("Dropping: ", amountL, "Litres")
    return None

def sendInfoToSA200(distance, angle):
    """Function which sends the bearing and heading information to SA200"""
    return None

def getWaterLevel():
    """Function which will recieve the value of the water level"""
    return None