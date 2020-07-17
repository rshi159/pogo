#motor controller for the pogo arm
import RPi.GPIO as GPIO
import time

servoPIN = 17 #top
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
GPIO.setup(23,GPIO.OUT) #bottom
GPIO.setup(22,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
hold = True

def convert_angle_to_duty(angle):
    return angle / 12 + 2

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
q = GPIO.PWM(23, 50)
r = GPIO.PWM(22, 50)
s = GPIO.PWM(27, 50)
# 20 ms cycle
# amazon says 1ms to 2ms width so between 5% and 10% duty?
#experimental gives 2% to 12% which is 0.4ms to 2.4ms
p.start(convert_angle_to_duty(0)) # Initialization0
q.start(convert_angle_to_duty(120)) #120
time.sleep(1)
r.start(convert_angle_to_duty(0))
s.start(convert_angle_to_duty(0))
time.sleep(1)
try:
    while True:
#     p.ChangeDutyCycle(convert_angle_to_duty(0))
#     q.ChangeDutyCycle(convert_angle_to_duty(120))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(30))
#     q.ChangeDutyCycle(convert_angle_to_duty(90))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(60))
#     q.ChangeDutyCycle(convert_angle_to_duty(60))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(90))
#     q.ChangeDutyCycle(convert_angle_to_duty(30))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(120))
#     q.ChangeDutyCycle(convert_angle_to_duty(0))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(90))
#     q.ChangeDutyCycle(convert_angle_to_duty(30))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(60))
#     q.ChangeDutyCycle(convert_angle_to_duty(60))
#     time.sleep(0.5)
#     p.ChangeDutyCycle(convert_angle_to_duty(30))
#     q.ChangeDutyCycle(convert_angle_to_duty(90))
#     time.sleep(0.5)
###smooth motion
#         for i in range(0,120,1):
#             p.ChangeDutyCycle(convert_angle_to_duty(i))
#             q.ChangeDutyCycle(convert_angle_to_duty(120-i))
#             time.sleep(0.005)
#         for i in range(0,120,1):
#             p.ChangeDutyCycle(convert_angle_to_duty(120-i))
#             q.ChangeDutyCycle(convert_angle_to_duty(i))
#             time.sleep(0.005)
        for i in range(0,120,1):
            if hold:
                p.ChangeDutyCycle(convert_angle_to_duty(i))
                q.ChangeDutyCycle(convert_angle_to_duty(120-i))
            else:
                p.ChangeDutyCycle(convert_angle_to_duty(i))
                q.ChangeDutyCycle(convert_angle_to_duty(120-i))
            time.sleep(0.015)
        time.sleep(0.5)
        for i in range(0,120,1):
            if hold:
                p.ChangeDutyCycle(convert_angle_to_duty(120-i))
                q.ChangeDutyCycle(convert_angle_to_duty(i))
            else:
                p.ChangeDutyCycle(convert_angle_to_duty(120-i))
                q.ChangeDutyCycle(convert_angle_to_duty(i))
            time.sleep(0.015)
        time.sleep(0.5)
###hold
#         p.ChangeDutyCycle(convert_angle_to_duty(60))
#         q.ChangeDutyCycle(convert_angle_to_duty(60))
except KeyboardInterrupt:
  p.stop()
  q.stop()
  r.stop()
  s.stop()
  GPIO.cleanup()

