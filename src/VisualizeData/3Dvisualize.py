#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

x = []
y = []
z = []

data = open("Try1.txt")

fig = plt.figure()
ax = Axes3D(fig)

for line in data.readlines():
    curline = line.strip().split(" ")
    floatLine = map(float,curline)
    x.append(floatLine[0])
    y.append(floatLine[1])
    z.append(floatLine[2])

ax.plot(x,y,z)
plt.show()
data.close()


