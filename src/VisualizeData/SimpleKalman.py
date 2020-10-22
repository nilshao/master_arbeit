#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

from filterpy.kalman import KalmanFilter
from mpl_toolkits.mplot3d import Axes3D

x = []
y = []
z = []

x_kf = []
y_kf = []
z_kf = []

kf = KalmanFilter (dim_x=2, dim_z=1)

data = open("simple01.txt")

fig = plt.figure()
ax = Axes3D(fig)

for line in data.readlines():
    curline = line.strip().split(" ")
    
    floatLine = map(float,curline)
    if len(floatLine)>1:
        x.append(floatLine[1])
        y.append(floatLine[2])
        z.append(floatLine[3])
    
ax.plot(x,y,z,c = 'green')
ax.scatter(x,y,z,c = 'red')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')


plt.show()
data.close()

