import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()



tello.streamon()


i = 1
tello.takeoff()
frame_read = tello.get_frame_read()

#tello.set_video_direction(Tello.CAMERA_DOWNWARD)

tello.move_up(200)
while i < 3:
    cv2.imwrite("pictures/picture" + str(i) + ".png", frame_read.frame)
    tello.rotate_clockwise(90)
    i = i + 1


tello.land()
