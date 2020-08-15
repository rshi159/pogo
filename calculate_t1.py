#!/usr/bin/env python
import numpy as np
from math import pi
import matplotlib.pyplot as plt

plot = True
#calculate theta 1.
#joint angles associated with corners of device
#bottom left, bottom right, top left, top right; angles [link1_top, link2]
calibration = np.array([[33.75, 72],
                [0, 102.75],
                [72, 69.5],
                [75.25, 106.25],
                [68.75,86.25],#0,25
                [71,98.25],#0,50
                [61.5,77.0],#25,0
                [58.25,94],#25,25
                [108.25,59.75],#25,50
                [81.25,52],#50,0
                [47,97],#50,25
                [45.25,114.75],#50,50
                [42.5,81],#75,0
                [35.5, 96.25],#75,25
                [26,112],#75,50
                [36.25,74.5],#100,0
                [25.75,91.75],#100,25
                [12.75,106.25]])#100,50

# L1 = 8.598#7.76
# L2 = 7.515#6.882
L1=8.500
L2=7.718

phone_x = 6.655
phone_y = 12.5
dist_x = 4.0

#same format as calibration
coordinates = np.array([[0, phone_y],
               [phone_x, phone_y],
               [0, 0],
               [phone_x, 0],
                [2.5,0],
                [5.0,0],
                [0,2.5],
                [2.5,2.5],
                [5.0,2.5],
                [0,5.0],
                [2.5,5.0],
                [5.0,5.0],
                [0,7.5],
                [2.5,7.5],
                [5.0,7.5],
                [0,10.0],
                [2.5,10.0],
                [5.0,10.0]])

remove = [1,3]
calculated = np.empty([len(calibration),2])
               
# convert radians to degrees
def rad_to_deg(arr):
    if np.all(np.abs(arr) <= 2*pi):
        return arr/(2*pi) * 360.0
    else:
        print("May Already Be In Degrees")
        return arr/(2*pi) * 360.0

# convert degrees to radians
def deg_to_rad(arr):
    if np.all(np.abs(arr) >= 2*pi):
        return (arr/360.0) * 2* pi
    else:
        print("May Already Be In Radians")
        return (arr/360.0) * 2* pi

#Offset to get T2_star from T2 - T2_offset
T2_star_offset = deg_to_rad(23.75)

if plot:
    plt.ion()
    plt.axis([-(L1+L2), L1+L2, -(L1+L2), L1+L2])

for i in range(len(calibration)):
    #planning angle for L2
    [T1_star,T2_star] = deg_to_rad(calibration[i])
    #distance from L1 base to end effector.
    c = np.sqrt(np.power(L1,2) + np.power(L2,2)-2*L1*L2*np.arccos(T2_star))

    #amount T1 is decreased by to set the vector c on the midline of the phone screen
    T1_offset = np.arcsin(L2*np.sin(T2_star))/c
    
    T2_star = T2_star - T2_star_offset
    #desired planning angle from actual angle of T1.
#     T1_star = T1 - T1_offset

    ###Calculate desired angle T1
    i_star = coordinates[i][0]
    j_star = coordinates[i][1]
    x = phone_x + dist_x - i_star
    y = 6.045 - j_star
    length_c = np.sqrt(np.sum(np.power(x,2) + np.power(y,2)))
    angle_c = np.arctan(y/x)
    
#     T2_expected = pi + T2_offset - np.arccos((np.power(length_c,2)-(np.power(L1,2)+np.power(L2,2)))/(-2*L1*L2))
    T2_expected = pi - np.arccos((np.power(length_c,2)-(np.power(L1,2)+np.power(L2,2)))/(-2*L1*L2))

#     print(pi+T2_offset)
#     print((np.power(length_c,2)-(np.power(L1,2)+np.power(L2,2)))/(-2*L1*L2))
#     print(np.arccos((np.power(length_c,2)-(np.power(L1,2)+np.power(L2,2)))/(-2*L1*L2)))
    T1_offset = np.arcsin(L2/length_c*np.sin(T2_expected))
    T1_expected = angle_c - T1_offset
    print("===========%d==============" % (i))
    print("i_star: %f" % (i_star))
    print("j_star: %f" % (j_star))
    print("x: %f" % (x))
    print("y: %f" % (y))
    print("angle_c: %f" % (angle_c))
    print("T1_offset: %f" % (T1_offset))
    print("T1_expected: %f" % (T1_expected))
    print("T2_expected: %f" % (T2_expected))
    print("T1_star: %f" % (T1_star))
    print("T2_star: %f" % (T2_star))
    calculated[i,0] = T1_expected
    calculated[i,1] = T2_expected
#     print("T1_actual: %f" % (T1_star))
#     print("T2_actual: %f" % (T2_star))
    if plot:
        L1_plot = [L1*np.cos(T1_expected), L1*np.sin(T1_expected)]
        rotation_mat = [[np.cos(T1_expected), -np.sin(T1_expected)],
                        [np.sin(T1_expected), np.cos(T1_expected)]]
        L2_plot = L1_plot + np.matmul(rotation_mat, [L2*np.cos(T2_expected), L2*np.sin(T2_expected)])
        plt.plot([0,L1_plot[0]], [0,L1_plot[1]])
        plt.plot([L1_plot[0], L2_plot[0]],[L1_plot[1], L2_plot[1]])
        plt.scatter(x,y)
        
        print(L1_plot+np.matmul(rotation_mat, [L2*np.cos(T2_expected), L2*np.sin(T2_expected)]))
        plt.pause(0.5)

T1_planning = np.array([calculated[:,0], np.ones([len(calibration)])]).T
T2_planning = np.array([calculated[:,1], np.ones([len(calibration)])]).T

T1_actual = np.array(deg_to_rad(calibration[:,0]))
T2_actual = np.array(deg_to_rad(calibration[:,1]))

T1_planning=np.delete(T1_planning, remove, 0)
T2_planning=np.delete(T2_planning, remove, 0)
T1_actual=np.delete(T1_actual,remove)
T2_actual=np.delete(T2_actual,remove)

print(T1_planning)
print(T1_actual)

T1_tf = np.matmul(np.matmul(np.linalg.pinv(np.matmul(T1_planning.T,T1_planning)),T1_planning.T),T1_actual)

T1_corrected = np.matmul(T1_planning,T1_tf)
print(T1_tf)
print(T1_corrected)

error = np.mean(np.abs(T1_actual - T1_corrected))
print("The error is: %f" %(error))

    
if plot:
    plt.draw()
    


