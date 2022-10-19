import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


#**********************************************************************
# Data Gathered From Testing Session
#**********************************************************************

smallL = 20
mediumL = 20
largeL = 40
log1L = 20
log2L = 20
mainL = 70

# With Thresholding to Zero before finding target
smallBefore = 145
smallAfter = 5
smallDiff = smallBefore-smallAfter

mediumBefore = 205
mediumAfter = 18
mediumDiff = mediumBefore-mediumAfter

largeBefore = 120
largeAfter = 7
largeDiff = largeBefore-largeAfter

log1Before = 140
log1After = 29
log1Diff = log1Before-log1After

log2Before = 103
log2After = 14
log2Diff = log2Before-log2After

diffThresh = [smallDiff, mediumDiff, largeDiff, log1Diff, log2Diff]
print(diffThresh)

# Without Thresholding to Zero before finding target
smallBefore = 199
smallAfter = 95
smallDiff = smallBefore-smallAfter

mediumBefore = 230
mediumAfter = 108
mediumDiff = mediumBefore-mediumAfter

largeBefore = 191
largeAfter = 103
largeDiff = largeBefore-largeAfter

log1Before = 201
log1After = 115
log1Diff = log1Before-log1After

log2Before = 180
log2After = 110
log2Diff = log2Before-log2After

mainBefore = 250
mainAfter = 180
mainDiff = mainBefore-mainAfter

diffNoThresh = [smallDiff, mediumDiff, largeDiff, log1Diff, log2Diff, mainDiff]

#**********************************************************************
# Forming Relationships for Kernel Size and Drop Size
#**********************************************************************
# let x = drop amount, y = kernel size
x = np.array([0, 20, 40, 60, 100])
y = np.array([0, 0.5, 0.9, 1.5, 2])
def kdiamModel(x, a, b, c):
    return a*x**2 + b*x + c

params, cov = curve_fit(kdiamModel, x, y)
yFit = kdiamModel(x, *params)
# print(params)
# plt.plot(x, yFit)
# plt.plot(x, y, '.')
# plt.show()

# print("est: ", kdiamModel(20, *params))
# let x = 