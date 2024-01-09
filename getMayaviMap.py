# encoding: utf-8
import mayavi.mlab as mlab
from numpy import exp,sin,cos,tan,random,mgrid,ogrid,linspace,sqrt,pi
import numpy as np
import matplotlib.pyplot as plt
mlab.figure(fgcolor=(0, 0, 0), bgcolor=(1, 1, 1)) #更改背景色

data = np.loadtxt('./output/error.txt' )
#print(data[:,0])

X=data[:,0]
Y=data[:,1]
Z=data[:,2]
error=data[:,3]
mlab.points3d(X, Y, Z, error, mode='cube', line_width=500)
mlab.colorbar()
mlab.show()
