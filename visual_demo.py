from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Make data
def spacial_line(x_s,x_e,y_s,y_e,z_s,z_e):
    x = np.linspace(x_s,x_e,100)
    y = np.linspace(y_s,y_e,100)
    z = np.linspace(z_s,z_e,100)
    # Plot the surface
    #ax.plot_surface(x, y, z, color='b')
    ax.plot(x, y, z, linewidth=1)

def viewing_box(list):
    for i in range(0,3):
        spacial_line(list[i][0],list[i+1][0],list[i][1],list[i+1][1],list[i][2],list[i+1][2])
    for i in range(4,7):
        spacial_line(list[i][0],list[i+1][0],list[i][1],list[i+1][1],list[i][2],list[i+1][2])
    for i in range(0,4):
        spacial_line(list[i][0],list[i+4][0],list[i][1],list[i+4][1],list[i][2],list[i+4][2])
    for i in [0,4]:
        spacial_line(list[i][0],list[i+3][0],list[i][1],list[i+3][1],list[i][2],list[i+3][2])

if __name__ == '__main__':
    bounding_list = [[0,0,0],
    [8,0,0],
    [8,5,0],
    [0,5,0],
    [2,3,8],
    [6,3,8],
    [6,4,8],
    [2,4,8]]

    viewing_box(bounding_list)
    plt.show()