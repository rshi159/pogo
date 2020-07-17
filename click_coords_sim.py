#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
# from scipy.integrate import trapz
from math import pi
import arm_ik as ik
# import pogo_arm_controller_v1.py as ctrl
import pogo_arm_controller_sim as ctrl


phone_x = 6.655
phone_y = 12.5

dist_x = 4.0
dist_y = phone_y/2

theta = ik.generate_rand_angles()

# coordinates of phone to coordinates of device
def convert_coords(coords):
    x = coords[0]
    y = coords[1]
    # new_x = phone_x + dist_x - x
    # new_y = y - dist_y
    t = deg_to_rad(60)
    print("theta: "+str(theta))
    rot_mat = np.array([[np.cos(t), -np.sin(t)],
                        [np.sin(t), np.cos(t)]])
    a = np.array(coords)
    new_x = np.dot(rot_mat[0], a)
    new_y = np.dot(rot_mat[1], a)
    return [new_x, new_y]

# convert degrees to radians
def deg_to_rad(arr):
    return (arr/360.0) * 2* pi

# Simple mouse click function to store coordinates
def onclick(event, debug = False):
    global ix, iy
    ix, iy = event.xdata, event.ydata

    # print 'x = %d, y = %d'%(
    #     ix, iy)

    # assign global variable to access outside of function
    global coords
    global theta
    global arm
    coords=[ix, iy]

    # print click
    print(coords)
    if coords[0] != None and coords[1] != None:
        if not np.isnan(coords).any():
            if debug:
                print("x: " + str(coords[0]))
                print("y: " + str(coords[1]))
                print("new_x: " + str(convert_coords(coords)[0]))
                print("new_y: " + str(convert_coords(coords)[1]))
            print(arm.pos)
            # soln = ik.ik([arm.angles[0],arm.angles[1]], convert_coords(coords))
            soln = ik.ik([0.001,0.001], convert_coords(coords))
            print(soln)
            theta = soln
            arm.move_joint([soln[0], soln[1], 0])

arm = ctrl.motorController(17,23,22,27)

fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.add_patch(patch.Rectangle((dist_x,-dist_y), phone_x, phone_y))
axes = plt.gca()
axes.set_xlim([dist_x,dist_x+phone_x])
axes.set_ylim([-dist_y,-dist_y+phone_y])

coords = []

# Call click func
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.show(1)