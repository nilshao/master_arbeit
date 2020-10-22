#!/usr/bin/env python

import matplotlib.pyplot as plt

x = []
y = []

data = open("DataExample.txt")

for line in data.readlines():
    curline = line.strip().split(" ")
    floatLine = map(float,curline)
    x.append(floatLine[0])
    y.append(floatLine[1])

data.close()

plt.plot(x,y)
plt.show()





