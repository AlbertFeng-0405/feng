from matplotlib import colors
import numpy as np
import matplotlib.pyplot as plt
import cv2
from matplotlib.path import Path
import matplotlib.patches as patches
import rdp

x_list = []
y_list = []

n = 10 # Number of possibly sharp edges
r = .7 # magnitude of the perturbation from the unit circle, 
# should be between 0 and 1
N = n*3+1 # number of points in the Path
# There is the initial point and 3 points per cubic bezier curve. Thus, the curve will only pass though n points, which will be the sharp edges, the other 2 modify the shape of the bezier curve

angles = np.linspace(0,2*np.pi,N)
codes = np.full(N,Path.CURVE4)
codes[0] = Path.MOVETO

verts = np.stack((np.cos(angles),np.sin(angles))).T*(2*r*np.random.random(N)+1-r)[:,None]
verts[-1,:] = verts[0,:]
print(verts) # Using this instad of Path.CLOSEPOLY avoids an innecessary straight line
# path = Path(verts, codes)

fig = plt.figure(figsize=(10,10))
plt.xlim(-5,5)
plt.ylim(-5,5)
# ax = fig.add_subplot(111)
# patch = patches.PathPatch(path, facecolor='none', lw=2)

# ax.set_xlim(np.min(verts)*1.1, np.max(verts)*1.1)
# ax.set_ylim(np.min(verts)*1.1, np.max(verts)*1.1)
# ax.axis('on') # removes the axis to leave only the shape
for i in verts[:len(verts)//2]:
    x_list.append(i[0])
    y_list.append(i[1])

plt.plot(x_list,y_list,'b')
x = np.array([0])
y = np.array([-3])

plt.scatter(x, y, c='r')

plt.grid()
plt.show()