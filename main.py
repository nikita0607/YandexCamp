from socket_conn import TCPSocket


from movement import Movement
from skills import Skills, Grabbing

host = "192.168.2.42"
port = 2001
TCPSocket.connect_to(host, port)
Movement.update_servo()
TCPSocket.sleep(0.1)

# Первая команда
speed = 60
Movement.set_speed(speed)

print("Load model...")
from detector import *
print("Model loaded!")

stream_url = 'http://192.168.2.42:8080/?action=stream'

print("Init camera")
Camera.init_cam(stream_url)
print("Camera inited")


print("Start move pipeline")
Skills.move_to_obj_pipe(3)

import keyboard_move
keyboard_move.init()
time.sleep(1)
# grab_sphere()
while 1:
    TCPSocket.update()
