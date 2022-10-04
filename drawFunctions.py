import cv2
import numpy as np
from CVfunctions import getLitres, contourCOM

def drawRefFrame(frame, width, height):
    """Draws the reference frame on the frame for visual aid"""
    size = 0.7
    thickness = 1
    colour = (0, 0, 0)
    gap = 40
    topMid = (int(width/2), 0)
    botMid = (int(width/2), height)
    leftMid = (0, int(height/2))
    rightMid = (width, int(height/2))
    cv2.line(frame, (topMid[0], topMid[1]+gap), (botMid[0], botMid[1]-gap), colour, thickness)
    cv2.line(frame, (leftMid[0]+gap+10, leftMid[1]), (rightMid[0]-gap-10, rightMid[1]), colour, thickness)
    cv2.putText(frame, '-0+', (topMid[0]-25, topMid[1]+30), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.putText(frame, '+90', (rightMid[0]-50, rightMid[1]+5), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.putText(frame, '-90', (leftMid[0], leftMid[1]+5), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.putText(frame, '-180+', (botMid[0]-40, botMid[1]-10), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    return frame

def drawTargetInfo(frameOut, targetLoc, dGuess, theta, weighting, width, height, Litres):
    size = 0.5
    thickness = 2
    colour = (0, 0, 0)
    Xadj = 20
    centerLoc = (int(width/2), int(height/2))
    weighting = int(weighting)
    cv2.circle(frameOut, targetLoc, 5, colour, thickness) 
    # cv2.putText(frameOut, str(int(dGuess))+'pix '+str(round(theta*180/np.pi, 1))+'deg', (targetLoc[0]+Xadj, targetLoc[1]+0), 
    #                                             cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    # if Litres:
    #     cv2.putText(frameOut, 'Drop: '+str(Litres) + 'L', (targetLoc[0]+Xadj, targetLoc[1]+25),
    #                                             cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    # else:
    #     cv2.putText(frameOut, 'Weight: '+str(weighting), (targetLoc[0]+Xadj, targetLoc[1]+25),
    #                                             cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    cv2.line(frameOut, centerLoc, targetLoc, colour, thickness)
    return frameOut

def drawScatteredWeights(frameDraw, frameValues, width, height, litres, i):
    """Display the weightings from a distribution of points in the frame"""
    thickness = 1
    colour = (0, 140, 255)
    size = 0.4
    N = 20
    Y = np.linspace(0, width, N, dtype=int)
    X = np.linspace(0, height, N, dtype=int)
    points = []
    cv2.putText(frameDraw, str(frameValues[X[-2]][Y[-2]]), (X[-1], Y[-4]), cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    for x in X[1:-1]:
        row = []
        for y in Y[1:-1]:
            newPoint = (y, x)
            if litres:
                newWeight = getLitres(frameValues[x][y])
            else:
                newWeight = frameValues[x][y]
            # if not i:
            #     point = newPoint
            #     weight = newWeight
            row.append(newPoint)
            cv2.putText(frameDraw, str(newWeight), newPoint, cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
        points.append(row)   
    return frameDraw

def drawCircles(frame, targetLoc, blurKsize, maskRadius):
    """Draws circles on the frame representing the Blur Kernel, 
        Target Localisation Mask and the Target Point"""
    thickness = 1
    colour = (0, 0, 0)
    cv2.circle(frame, targetLoc, 3, colour, thickness)
    cv2.circle(frame, targetLoc, blurKsize, colour, thickness)
    cv2.circle(frame, targetLoc, maskRadius, colour, thickness)
    return frame

def drawContours(frame, contourList, thicknessList):
    """Draws a contours from a list with specified thicknesses"""
    colour = (0, 140, 255)
    for j in range(0, len(contourList)):
        contours = contourList[j]
        for i in range(0, len(contours)):
            cv2.drawContours(frame, [contours[i].astype(int)], 0, (0, 140, 255), thicknessList[j])
    return frame

def drawContourAreas(frame, contours, areas):
    """Draws the areas of contours positioned at their COM"""
    thickness = 2
    colour = (0, 0, 0)  
    size = 0.5
    for i in range(0, len(contours)):
        COM = contourCOM(contours[i])
        cv2.putText(frame, ". Area: "+str(areas[i]), COM, cv2.FONT_HERSHEY_SIMPLEX, size, colour, thickness)
    return frame