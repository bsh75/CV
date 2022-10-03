import numpy as np
# from geoLocationFunctions import distApprox


def getDroneHeight():
    """Returns the height of the drone above point in center of frame in practice 
    this will be the function that recieves height from drone onboard computer"""
    height = 2.4 #m
    return height

def GIMBALsendTarget(pixDistance, angle, frameWidth, frameHeight):
    """Send the bearing and heading to GIMBAL and return an actual distance calculation"""
    # For now just use trigonometric flat ground approximation
    distance = 5# distApprox(pixDistance, angle, frameWidth, frameHeight)
    return distance

def dropWater(targetVal):
    """Send the drop command to the valve to release water"""
    amountL = targetVal/100
    print("DROPPING - DROPPING - DROPPING - DROPPING - DROPPING - DROPPING")
    return None