import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

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
