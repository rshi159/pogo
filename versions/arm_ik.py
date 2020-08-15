#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from math import pi

# phonex = 6.55
# phoney = 12.5
# x_offset = 11.4
# y_offset = 8.2
# 
#  #length of links
# L1 = 7.76
# L2 = 6.882
phonex = 6.8
phoney = 12.63
x_offset = 10.6
y_offset = phonex/2

 #length of links
L1 = 8.00
L2 = 7.42

 # top left corner of screen is 0,0.
lim = L1+L2

 # max iterations of cyclic coordinate descent
MAX_ITER = 15
 # termination threshold
THRESH = 0.5
 # number of links
NUM_LINK = 2
# debug
debug = not True

# function [x,y] = generate_rand_coords()
def generate_rand_coords():
    global L1
    global L2
    x = np.random.rand() * (L1+L2)
    y = np.random.rand() * np.sqrt(np.power((L1+L2),2) - np.power(x,2))
    return [x, y]
# end

# function [t1,t2] = generate_rand_angles()
def generate_rand_angles():
    t1 = np.random.rand(1) * 2 * pi
    t2 = np.random.rand(1) * 2 * pi
    return [t1, t2]
# end
# function [coords] = fk(arr)
def fk(arr):
    global L1
    global L2
    a = arr[0]
    b = arr[1]
    p1 = np.array([L1*np.cos(a), L1*np.sin(a)])
    p2 = np.array([L2*np.cos(b), L2*np.sin(b)])
    coords = np.array([[0,0],
              [p1[0], p1[1]],
              [p1[0]+p2[0], p1[1]+p2[1]]])
    return coords
# end

# function a = mag(arr)
def mag(arr):
    a = np.sqrt(np.sum(arr*arr))
    return a

# function dist = check_dist(a,b)
def check_dist(a,b):
    dist = mag(a-b)
    return dist

# take in starting joint angle and desired end effector position.
# returns joint angles for desired position
def ik(start,desired):
    target = [desired[0], desired[1]]
    print("target: "+str(target))
    initial = [start[0], start[1]]
    start_coord = fk(initial)
    print(start_coord)
    if debug:
        print("start angles:\n" + str(start))
        print("start_coordinates:\n" + str(start_coord))
        print("target:\n"+ str(target))

 ##Start cyclic coordinate descent
 #  position of links
    pos2 = np.array([start_coord[2,0], start_coord[2,1]])
    pos1 = np.array([start_coord[1,0], start_coord[1,1]])
    pos0 = np.array([0, 0])
    print("pos0: "+str(pos0))
    print("pos1: "+str(pos1))
    print("pos2: "+str(pos2))
    pos = np.array([[pos0], [pos1], [pos2]])
 #     vector representing links. base of link1 is the origin
    vec2 = pos2 - pos1
    vec1 = pos2
    vec = np.array([vec1/mag(vec1), vec2/mag(vec2)])
    theta1 = initial[0]
    theta2 = initial[1]
    theta = np.array([theta1, theta2])
 #     handle breaking from nested loops
    flag = 0
 #     start iterating over entire kinematic tree from outer most link
    for k in np.arange(0,MAX_ITER):
        if flag == 1:
            break
 #       termination condition        
        for j in np.arange(NUM_LINK-1,-1, -1):  
            if  check_dist(pos[2], target) <= THRESH:
                if debug:
                    print(">>Reached<<")
                flag = 1
                break
 #         move link
            des = (target - pos[j])[0]
            des = des / mag(des)
            d = np.dot(vec[j,:], des)
            t2_correct = np.arccos(d/(mag(vec[j,:])*mag(des)))
 #          direction to rotate  
            c = np.cross(np.append(des,0), np.append(vec[j,:],0))
            dir = np.sign(c[2])   
            theta[j]  = theta[j] - dir*t2_correct
            # if theta[j] >= 2 * pi:
            #     theta[j] = theta[j] - (2*pi)
            # if theta[j] < 0:
            #     theta[j] = theta[j] + (2*pi)
 #          find new fk
            coords = fk(theta)
            pos2 = np.array([coords[2,0], coords[2,1]])
            pos1 = np.array([coords[1,0], coords[1,1]])
            pos0 = np.array([0, 0])
            pos = np.array([[pos0], [pos1], [pos2]])
        #     vector representing links. base of link1 is the origin
            vec2 = pos2 - pos1
            vec1 = pos2
            vec = np.array([vec1/mag(vec1), vec2/mag(vec2)])
    if debug:
        print("final_coordinates:\n" + str(pos))
    return theta
# print("number success: "+ str(succ))
# print("number trials: "+ str(num_trials))