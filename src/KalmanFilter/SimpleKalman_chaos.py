#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

from filterpy.kalman import KalmanFilter
from mpl_toolkits.mplot3d import Axes3D
from filterpy.common import Q_discrete_white_noise

if __name__ == '__main__':
    # append x,y,z original and x,y,z after filtering in order to visualize later
    x_ori = []
    y_ori = []
    z_ori = []

    x_kf = []
    y_kf = []
    z_kf = []

    # define kalman filter: x is condition vector, z is observation vector
    kf = KalmanFilter (dim_x = 6, dim_z = 3)

    # read the file
    data = open("Try1.txt")

    #initialize plotting
    fig = plt.figure()
    ax = Axes3D(fig)

    #kf state transition matrix
    kf.F = np.array([[1.,0.,0.,1.,0.,0.],
                    [0.,1.,0.,0.,1.,0.],
                    [0.,0.,1.,0.,0.,1.],
                    [0.,0.,0.,1.,0.,0.],
                    [0.,0.,0.,0.,1.,0.],
                    [0.,0.,0.,0.,0.,1.]])
    #kf measurement function:
    kf.H = np.array([[1.,0.,0.,0.,0.,0.],
                    [0.,1.,0.,0.,0.,0.],
                    [0.,0.,1.,0.,0.,0.]])
    #kf covariance matrix: P already contains np.eye, multiplies uncertainty
    kf.P *= 0.0001
    #measurement noice:
    kf.R = 0.0001                   #should think and tune how much error might have 

    #process noise         https://filterpy.readthedocs.io/en/latest/common/common.html
    kf.Q = Q_discrete_white_noise(dim = 2, dt=0.03, var=1., block_size=3)

    for line in data.readlines():

        # split the information by space
        curline = line.strip().split(" ")
        floatLine = np.array([float(x) for x in curline])
        
        # and save the information in a map
       # floatLine = map(float,curline)
        
        #if there is coordinate in this line
        if len(floatLine)>1:

            #kf initialization
            if len(x_kf) == 0:
                x_kf.append(floatLine[1])
                
                y_kf.append(floatLine[2])
                z_kf.append(floatLine[3])
                
                #xt = [Tx,Ty,Tz,Tx.dot,Ty.dot,Tz.dot]
                kf.x = np.array([[x_kf[0]],[y_kf[0]],[z_kf[0]],[0.],[0.],[0.]])

            else:
                #zt is the observation vector
                zt = np.array([floatLine[1],floatLine[2],floatLine[3]])
                
                kf.predict()
                kf.update(zt)

                x_kf.append(kf.x[0][0])
                y_kf.append(kf.x[1][0])
                z_kf.append(kf.x[2][0])

            x_ori.append(floatLine[1])
            y_ori.append(floatLine[2])
            z_ori.append(floatLine[3]) 
    #plot is a line, scatter are discrete points
    ax.plot(x_ori,y_ori,z_ori,c = 'green')
    ax.scatter(x_ori,y_ori,z_ori,c = 'green')
    
    ax.plot(x_kf,y_kf,z_kf,c = 'red')
    ax.scatter(x_kf,y_kf,z_kf,c = 'red')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    plt.show()
    data.close()

    #add lable
    

    

