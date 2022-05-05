import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plyfile
from plyfile import PlyData

vertices = np.zeros([3,8],dtype=int)
vertices[0,:] = [1, 7, 5, 8, 2, 4, 6, 3]
vertices[1,:] = [1, 7, 4, 6, 8, 2, 5, 3]
vertices[2,:] = [6, 1, 5, 2, 8, 3, 7, 4]
vertices = vertices - 1 #(adjust the indices by one since python starts with zero indexing)

# Define an array with dimensions 8 by 3
# 8 for each vertex
# -> indices will be vertex1=0, v2=1, v3=2 ...
# 3 for each coordinate
# -> indices will be x=0,y=1,z=1
object = PlyData.read('example.ply')

# Define x values
#object[:,0] = [0, 0, 0, 0, 1, 1, 1, 1]
# Define y values
#object[:,1] = [0, 1, 0, 1, 0, 1, 0, 1]
# Define z values
#object[:,2] = [0, 0, 1, 1, 0, 0, 1, 1]

# First initialize the fig variable to a figure
fig = plt.object()
# Add a 3d axis to the figure
ax = fig.add_subplot(111, projection='3d')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.axis("off")

azimuth = 0
elevation = 0
roll = 0

def update(frame_number):
    global azimuth
    global elevation
    global roll
    azimuth += 5
    elevation += 5
    roll += 5
    ax.set_title("azimuth: " + str(azimuth) + " / elevation: " + str(elevation) + " / roll: " + str(roll))
    ax.view_init(azim=azimuth, elev=elevation, roll=roll)

# Construct the animation, using the update function as the animation director.
animation = FuncAnimation(fig, update) # add interval=1000 if necessary

plt.show()

# Links:
# 
# https://stackoverflow.com/questions/43180357/how-to-rotate-a-3d-plot-in-python-or-as-a-animation-rotate-3-d-view-using-mou
# https://github.com/matplotlib/matplotlib/pull/21426
# https://stackoverflow.com/questions/70911608/plot-3d-cube-and-draw-line-on-3d-in-python
