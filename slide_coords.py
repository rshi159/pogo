#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
# from scipy.integrate import trapz
from math import pi
#import arm_ik as ik
# import pogo_arm_controller_v1.py as ctrl
#import pogo_arm_controller_v1 as ctrl
from matplotlib.widgets import Slider, Button, RadioButtons
#motor controller for the pogo arm
import RPi.GPIO as GPIO
import time



# convert degrees to radians
def deg_to_rad(arr):
    return (arr/360.0) * 2* pi


##########
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(17,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)

def convert_angle_to_duty(angle):
    return angle / 12 + 2

p = GPIO.PWM(23, 50) # GPIO 17 for PWM with 50Hz
q = GPIO.PWM(17, 50)
r = GPIO.PWM(27, 50)
s = GPIO.PWM(22, 50)
# 20 ms cycle
# amazon says 1ms to 2ms width so between 5% and 10% duty?
#experimental gives 2% to 12% which is 0.4ms to 2.4ms
p.start(convert_angle_to_duty(0)) # Initialization0
q.start(convert_angle_to_duty(120)) #120
time.sleep(1)
r.start(convert_angle_to_duty(0))
s.start(convert_angle_to_duty(0))
time.sleep(1)

####
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
delta = 0.25
L1t = 120
L1b = 0
L2 = 0
L3 = 0

axcolor = 'lightgoldenrodyellow'
axL1t = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
axL1b = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
axL2 = plt.axes([0.25, 0.2, 0.65, 0.03], facecolor=axcolor)

sL1t = Slider(axL1t, 'Top', 0, 120, valinit=L1t, valstep=delta)
sL1b = Slider(axL1b, 'Bottom', 0, 120, valinit=L1b, valstep=delta)
sL2 = Slider(axL2, 'link2', 0, 120, valinit=L1b, valstep=delta)

def update(val):
    #L1t = sL1t.val
    L1b = sL1b.val
    L1t = 120-sL1b.val
    L2 = sL2.val
    p.ChangeDutyCycle(convert_angle_to_duty(L1b))
    q.ChangeDutyCycle(convert_angle_to_duty(L1t))
    r.ChangeDutyCycle(convert_angle_to_duty(L2))
    time.sleep(1)
    print("L1t = %f.2" % L1t)
    print("L1b = %f.2" % L1b)
    print("L2 = %f.2" % L2)
    #set the motor controller to these above values


sL1t.on_changed(update)
sL1b.on_changed(update)
sL2.on_changed(update)

resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


def reset(event):
    sL1b.reset()
    sL1t.reset()
    sL2.reset()
button.on_clicked(reset)

rax = plt.axes([0.025, 0.5, 0.15, 0.15], facecolor=axcolor)
radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)

def colorfunc(label):
    fig.canvas.draw_idle()
radio.on_clicked(colorfunc)

plt.show()

try:
    plt.show()
    p.stop()
    q.stop()
    r.stop()
    s.stop()
    GPIO.cleanup()

except KeyboardInterrupt:
    print("Problem")
    plt.close()
    p.stop()
    q.stop()
    r.stop()
    s.stop()
    GPIO.cleanup()
