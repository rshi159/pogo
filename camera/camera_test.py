from picamera import PiCamera
#from time import sleep

camera = PiCamera()

camera.resolution = (1296,972)
camera.start_preview()
input()
camera.stop_preview()