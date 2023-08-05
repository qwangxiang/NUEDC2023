import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()

tello.send_command_with_return("downvision 1")

tello.streamon()

frame_read = tello.get_frame_read()


tello.takeoff()
cv2.imwrite("picture.png", frame_read.frame)

tello.land()
