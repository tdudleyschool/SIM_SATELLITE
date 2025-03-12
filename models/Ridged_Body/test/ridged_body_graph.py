import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

rotation_matrix = np.loadtxt('../rotation1_output.txt', delimiter =', ')
omega_vec = np.loadtxt('../omega_output.txt', delimiter = ', ')

#rotation_matrix = df_r.values
#omega_vec = df_w.values

axes = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

rotated_axis = np.dot(rotation_matrix,axes.T).T
print(rotated_axis)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.view_init(elev=15, azim=75)
ax.set_xlim(left=0)
ax.set_ylim(bottom=0)
ax.set_zlim(bottom=0)
#original axis
ax.quiver(0, 0, 0, axes[0, 0], axes[0, 1], axes[0,2], color='r', linestyle = '--', linewidth = 3,  label='x')
ax.quiver(0, 0, 0, axes[1, 0], axes[1, 1], axes[1,2], color='g', linestyle = '--', linewidth = 3, label='y')
ax.quiver(0, 0, 0, axes[2, 0], axes[2, 1], axes[2,2], color='b', linewidth = 3, linestyle = '--', label='z')

#new rotated axis
ax.quiver(0, 0, 0, rotated_axis[0, 0], rotated_axis[0, 1], rotated_axis[0,2], color='magenta', alpha = 0.5, linewidth = 4, linestyle='-', label="x'")
ax.quiver(0, 0, 0, rotated_axis[1, 0], rotated_axis[1, 1], rotated_axis[1,2], color='lightgreen', alpha = 0.5, linewidth = 4, linestyle='-', label="x'")
ax.quiver(0, 0, 0, rotated_axis[2, 0], rotated_axis[2, 1], rotated_axis[2,2], color='b', alpha = 0.5, linewidth = 4, linestyle='-', label="x'")

#omega vector
ax.quiver(0, 0, 0, omega_vec[0], omega_vec[1], omega_vec[2], color='y', linewidth = 4, alpha = 0.6, linestyle = '-', label = "w vec")

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Rotation About Vector Ex: 2")

plt.savefig('Rotation Vec2 .png')