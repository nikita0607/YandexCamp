import time

from movement import Movement
import keyboard as kb

from socket_conn import TCPSocket


def init():
    print("Init keyboard movement")
    kb.add_hotkey('w', lambda: Movement().move_sync(1))
    kb.add_hotkey('s', lambda: Movement().move_sync(2))
    kb.add_hotkey('a', lambda: (Movement.set_speed_l(20), Movement.set_speed_r(40)))
    kb.add_hotkey('d', lambda: Movement().move(4, 0))

    kb.add_hotkey('up', lambda: Movement().move_first_servo(2))
    kb.add_hotkey('down', lambda: Movement().move_first_servo(-2))

    kb.add_hotkey(';', lambda: Movement().move_second_servo(2))
    kb.add_hotkey('\'', lambda: Movement().move_second_servo(-2))

    kb.add_hotkey('[', lambda: Movement().rotate_servo(2))
    kb.add_hotkey(']', lambda: Movement().rotate_servo(-2))

    kb.add_hotkey('z', lambda: Movement().hand_servo(2))
    kb.add_hotkey('x', lambda: Movement().hand_servo(-2))

    kb.add_hotkey('space', lambda: Movement().move(0, 0))

    kb.add_hotkey('1', lambda: (Movement.set_speed_l(20), Movement.set_speed_r(20)))
    kb.add_hotkey('0', lambda: Movement.set_speed(100))
    print('KB movement was init.')

if __name__ == '__main__':
    host = "192.168.2.42"
    port = 2001
    TCPSocket.connect_to(host, port)
    init()
    while 1:
        TCPSocket.sleep(0.1)