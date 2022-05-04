import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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
cube = np.zeros([8,3])

# Define x values
cube[:,0] = [0, 0, 0, 0, 1, 1, 1, 1]
# Define y values
cube[:,1] = [0, 1, 0, 1, 0, 1, 0, 1]
# Define z values
cube[:,2] = [0, 0, 1, 1, 0, 0, 1, 1]

# First initialize the fig variable to a figure
fig = plt.figure()
fig2 = plt.figure()
# Add a 3d axis to the figure
ax = fig.add_subplot(111, projection='3d')
ax2 = fig2.add_subplot(111, projection='3d')

# plotting cube
# Initialize a list of vertex coordinates for each face
# faces = [np.zeros([5,3])]*3
faces = []
faces.append(np.zeros([5,3]))
faces.append(np.zeros([5,3]))
faces.append(np.zeros([5,3]))
faces.append(np.zeros([5,3]))
faces.append(np.zeros([5,3]))
faces.append(np.zeros([5,3]))
# Bottom face
faces[0][:,0] = [0,0,1,1,0]
faces[0][:,1] = [0,1,1,0,0]
faces[0][:,2] = [0,0,0,0,0]
# Top face
faces[1][:,0] = [0,0,1,1,0]
faces[1][:,1] = [0,1,1,0,0]
faces[1][:,2] = [1,1,1,1,1]
# Left Face
faces[2][:,0] = [0,0,0,0,0]
faces[2][:,1] = [0,1,1,0,0]
faces[2][:,2] = [0,0,1,1,0]
# Left Face
faces[3][:,0] = [1,1,1,1,1]
faces[3][:,1] = [0,1,1,0,0]
faces[3][:,2] = [0,0,1,1,0]
# front face
faces[4][:,0] = [0,1,1,0,0]
faces[4][:,1] = [0,0,0,0,0]
faces[4][:,2] = [0,0,1,1,0]
# front face
faces[5][:,0] = [0,1,1,0,0]
faces[5][:,1] = [1,1,1,1,1]
faces[5][:,2] = [0,0,1,1,0]
ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='k', alpha=.25))
ax2.add_collection3d(Poly3DCollection(faces, facecolors='red', linewidths=1, edgecolors='k', alpha=.25))

# plotting lines
ax.plot(cube[vertices[0,:],0],cube[vertices[0,:],1],cube[vertices[0,:],2],color='b')
ax.plot(cube[vertices[1,:],0],cube[vertices[1,:],1],cube[vertices[1,:],2],color='b')
ax.plot(cube[vertices[2,:],0],cube[vertices[2,:],1],cube[vertices[2,:],2],color='b')
ax2.plot(cube[vertices[0,:],0],cube[vertices[0,:],1],cube[vertices[0,:],2],color='r')
ax2.plot(cube[vertices[1,:],0],cube[vertices[1,:],1],cube[vertices[1,:],2],color='r')
ax2.plot(cube[vertices[2,:],0],cube[vertices[2,:],1],cube[vertices[2,:],2],color='r')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

ax.axis("off")
ax2.axis("off")

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

def update2(frame_number):
    global azimuth
    global elevation
    global roll
    azimuth += 5
    elevation += 5
    roll += 5
    ax2.set_title("azimuth: " + str(azimuth) + " / elevation: " + str(elevation) + " / roll: " + str(roll))
    ax2.view_init(azim=azimuth, elev=elevation, roll=roll)

# Construct the animation, using the update function as the animation director.
animation = FuncAnimation(fig, update) # add interval=1000 if necessary
animation2 = FuncAnimation(fig2, update2) # add interval=1000 if necessary

plt.show()

# Links:
# 
# https://stackoverflow.com/questions/43180357/how-to-rotate-a-3d-plot-in-python-or-as-a-animation-rotate-3-d-view-using-mou
# https://github.com/matplotlib/matplotlib/pull/21426
# https://stackoverflow.com/questions/70911608/plot-3d-cube-and-draw-line-on-3d-in-python