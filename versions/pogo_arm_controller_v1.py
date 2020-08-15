#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
from math import pi
import numpy as np
import matplotlib.pyplot as plt

'''
Runs the arm. Planning for the arm uses reference frame of the device. We have to change the coordinated by rotating.
'''
class motorController:
    def __init__(self, link1T_PIN, link1B_PIN, link2_PIN, link3_PIN):
        # set GPIO mode
        GPIO.setmode(GPIO.BCM)
        # set pins
        GPIO.setup(link1T_PIN, GPIO.OUT)
        GPIO.setup(link1B_PIN, GPIO.OUT)
        GPIO.setup(link2_PIN, GPIO.OUT)
        GPIO.setup(link3_PIN, GPIO.OUT)
        # set PWM
        self.link1T_PIN = GPIO.PWM(link1T_PIN, 50)
        self.link1B_PIN = GPIO.PWM(link1B_PIN, 50)
        self.link2_PIN = GPIO.PWM(link2_PIN, 50)
        self.link3_PIN = GPIO.PWM(link3_PIN, 50)
        # set joint lengths
        self.L1 = 7.76
        self.L2 = 6.882
        # set default link angle positions. in radians
        self.angles = np.array([2*pi/3,0,0,0])
        #start pins
        self.link1T_PIN.start(self.convert_angle_to_duty(self.rad_to_deg(self.angles[0])))
        self.link1B_PIN.start(self.convert_angle_to_duty(self.rad_to_deg(self.angles[1])))
        self.link2_PIN.start(self.convert_angle_to_duty(self.rad_to_deg(self.angles[2])))
        self.link3_PIN.start(self.convert_angle_to_duty(self.rad_to_deg(self.angles[3])))
        time.sleep(1)
        # set initial ee position
        self.pos = self.fk(self.angles)

    # remembers the joint angles. Takes joint angles in radians
    def setJointAngle(self,arr):
        self.angles = arr
    
    # remember the joint positions. Takes joint angles in radians
    def setJointPos(self, arr):
        self.pos = self.fk(arr)

    # moves to joint angles. Takes joint angles in radians
    def move_joint(self, arr, debug = False, make_plot=False, actuate=not False):
        steps = 25
        vel = 150
        # offset between planning 0 and actual robot zero in planning angles.
        zero_offset = 70.0*22.64/15.0
        #scale factor from planning to robot angles
        scale_p_to_r = 15/22.64
        # convert link angles to robot angles in radians
        robot_angles = self.convert_to_rob_angles(arr)
        target_pos = self.fk(arr)
        if debug:
            print("arr" + str(arr))
            print("robot_angles" + str(robot_angles))
            print("target" + str(target_pos))
            print("pos" + str(self.pos))
            print("diff" + str(target_pos - self.pos))
            print((target_pos - self.pos)[1:,:])
        dist = self.mag((target_pos[2] - self.pos[2]))
        dt = (dist/vel)/steps
        ds = (robot_angles - self.angles)/steps
        if debug:
            print("dist: " + str(dist))
            print("dt: " + str(dt))
            print("ds: " + str(ds))
        if make_plot:
            # plotting
            plt.ion()
            fig, ax = plt.subplots()
            ax = fig.add_subplot(111)
            axes = plt.gca()
            lim = self.L1 + self.L2
            axes.set_xlim([-lim,lim])
            axes.set_ylim([-lim,lim])
            plt.scatter(target_pos[2,0], target_pos[2,1])
            plt.scatter(self.fk(robot_angles[1:3])[:,0],self.fk(robot_angles[1:3])[:,1],)
        if debug:
            print(self.fk(robot_angles[1:3]))
        for i in np.arange(steps):
#             print("sa: "+str(self.angles))
            new_step = self.angles + ds
#             print("=======")
#             print(new_step)
#             print(self.rad_to_deg(new_step[1]))
#             print(180-self.rad_to_deg(new_step[2]-new_step[1])-50)
#             print(self.convert_angle_to_duty(self.rad_to_deg(new_step[1])))
#             print(self.convert_angle_to_duty(180-self.rad_to_deg(new_step[2]-new_step[1])-50))
#             print(self.convert_angle_to_duty(self.rad_to_deg(new_step[0])))
#             print(self.convert_angle_to_duty(self.rad_to_deg(new_step[2])))
            if make_plot:
                print("========")
                print(new_step)
                print("duty: ")
                #for link 1
                #we convert radians to degrees for the joint angles in planning space.
                #then we apply the zero offset since the origin for planning is at the midline of the phone.
                #    the zero for the robot is zero_offset degrees off for link 1.
                #then we convert planning angles to robot angles. This is only the case for link1, since the 2 servos are in opposite
                #    directions so they are constantly fighting. Need to add code to correct this possibly.
                print(self.convert_angle_to_duty(scale_p_to_r*(self.rad_to_deg(new_step[0])-zero_offset)))
                print(self.convert_angle_to_duty(scale_p_to_r*(self.rad_to_deg(new_step[1])+zero_offset)))
                #    the angle here is the same in planning and task space.
                print(self.convert_angle_to_duty(120-(180-self.rad_to_deg(new_step[2]-new_step[1])-50)))
                print(self.convert_angle_to_duty(self.rad_to_deg(new_step[3])))
                print("angles")
                print(scale_p_to_r*(self.rad_to_deg(new_step[0])-zero_offset))
                print(scale_p_to_r*(self.rad_to_deg(new_step[1])+zero_offset))
                print((self.rad_to_deg(new_step[2]-new_step[1])))
                print(self.rad_to_deg(new_step[3]))
            if actuate:
                self.link1T_PIN.ChangeDutyCycle(self.convert_angle_to_duty(scale_p_to_r*(self.rad_to_deg(new_step[0])-zero_offset)))
                self.link1B_PIN.ChangeDutyCycle(self.convert_angle_to_duty(scale_p_to_r*(self.rad_to_deg(new_step[1])+zero_offset)))
                self.link2_PIN.ChangeDutyCycle(self.convert_angle_to_duty(120-(180-self.rad_to_deg(new_step[2]-new_step[1])-50)))
                self.link3_PIN.ChangeDutyCycle(self.convert_angle_to_duty(self.rad_to_deg(new_step[3])))
            if make_plot:
                f = self.fk(np.array([new_step[1],new_step[2]]))
                plt.plot(f[:,0], f[:,1])
                plt.pause(0.01)
                plt.draw()
            self.setJointAngle(new_step)
            time.sleep(dt)
        # plt.close()
        # self.link1T_PIN.ChangeDutyCycle(robot_angles[0])
        # self.link1B_PIN.ChangeDutyCycle(robot_angles[1])
        # self.link2_PIN.ChangeDutyCycle(robot_angles[2])
        # self.link3_PIN.ChangeDutyCycle(robot_angles[3])
#         print(robot_angles)
#         print(arr)
#         print(target_pos)
#         print(f)
        self.setJointPos(arr)
        if make_plot:
            plt.plot(self.pos[:,0], self.pos[:,1])
            print("b")
            plt.pause(1)
            plt.close()


    # convert planned angle to actual angle (in radians)
#     def convert_to_rob_angles(self,arr):
#         t1b = arr[0]# + self.deg_to_rad(150)
#         t1t = self.deg_to_rad(120) - t1b
#         t2 = arr[1]# + self.deg_to_rad(30)
#         t3 = arr[2]
#         return [t1t, t1b, t2, t3]
    def convert_to_rob_angles(self,arr):
        arr = arr
        t1b = arr[0]# + self.deg_to_rad(150)
        t1t = self.deg_to_rad(181.12) - t1b
        t2 = arr[1]# + self.deg_to_rad(30)
        t3 = arr[2]
        return [t1t, t1b, t2, t3]

    # convert radians to degrees
    def rad_to_deg(self,arr):
        if np.all(abs(arr) <= 2*pi):
            return arr/(2*pi) * 360.0
        else:
            print("May Already Be In Degrees")
            return arr/(2*pi) * 360.0

    # convert degrees to radians
    def deg_to_rad(self,arr):
        if np.all(abs(arr) >= 2*pi):
            return (arr/360.0) * 2* pi
        else:
            print("May Already Be In Radians")
            return (arr/360.0) * 2* pi

    # convert angle in degrees to duty
    def convert_angle_to_duty(self,angle):
        return angle/12.0 + 2

    # fk. Takes joint angles in radians
    def fk(self, arr):
        L1 = self.L1
        L2 = self.L2
        a = arr[0]
        b = arr[1]
        p1 = np.array([L1*np.cos(a), L1*np.sin(a)])
        p2 = np.array([L2*np.cos(b), L2*np.sin(b)])
        coords = np.array([[0,0],
                [p1[0], p1[1]],
                [p1[0]+p2[0], p1[1]+p2[1]]])
        return coords
    
    # function a = mag(arr)
    def mag(self,arr):
        a = np.sqrt(np.sum(arr*arr))
        return a

    # function dist = check_dist(a,b)
    def check_dist(self,a,b):
        dist = self.mag(a-b)
        return dist

    # close gpio
    def cleanup(self):
        print("Cleaned")
        self.link1T_PIN.stop()
        self.link1B_PIN.stop()
        self.link2_PIN.stop()
        self.link3_PIN.stop()
        GPIO.cleanup()




# except KeyboardInterrupt:
#   p.stop()
#   GPIO.cleanup()