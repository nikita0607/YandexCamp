from torchvision.models.segmentation import DeepLabV3

from socket_conn import TCPSocket


from movement import Movement, DefaultAngles
from skills import Skills, Grabbing

host = "192.168.2.42"
port = 2001
TCPSocket.connect_to(host, port)
Movement.update_servo()
TCPSocket.sleep(0.1)

# Первая команда
speed = 40
Movement.set_speed(speed)
TCPSocket.send_buf(b'\xff\x06\x01\x00\xff')

print("Load model...")
from detector import *
print("Model loaded!")

stream_url = 'http://192.168.2.42:8080/?action=snapshot'

print("Init camera")
cam = Camera.init_cam(stream_url)
print("Camera inited")


print("Start move pipeline")


# Skills.move_to_obj_pipe(4)
# if Detector.is_object_visible(4):
#     Grabbing.press_button()
# else:
#     TCPSocket.sleep(0.2)


rep = 1
while rep:
    Skills.move_to_obj_pipe(3, ady=40)
    if Detector.is_object_visible(3):
        Grabbing.grab_obj(3)
    else:
        print("NOT SEEE#!!")
    TCPSocket.sleep(2)
    rep = Detector.is_object_visible(3)
    print("AA")

angls = (DefaultAngles.CAM_DROP_BASKET_PITCH+5, DefaultAngles.CAM_CUBE_GRAB_ROTATE)
Skills.move_to_obj_pipe(1, 40, ady=40, start_cam_pose=angls)
Skills.move_to_obj_pipe(1, 40, ady=40)
if Detector.is_object_visible(1):
    Movement.cam_pitch = DefaultAngles.CAM_DROP_BASKET_PITCH
    Movement.update_servo()
    TCPSocket.update()

    if Detector.is_object_visible(1):
        Grabbing.drop_to_basket()
    else:
        print("NOT SEEE#!!")
else:
    print("NOT SEEE#!!")

import keyboard_move
keyboard_move.init()
time.sleep(1)
# grab_sphere()
while 1:
    TCPSocket.update()
