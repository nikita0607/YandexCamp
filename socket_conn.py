import socket
import threading
import time

from collections import deque

import numpy as np

commands = deque()


class TCPSocket(socket.socket):
    inst = None
    recv_callbacks = []

    def __new__(cls, *args, **kwargs):
        if cls.inst:
            return cls.inst

        cls.inst = super().__new__(cls, *args, **kwargs)

        return cls.inst

    @classmethod
    def connect_to(cls, host, port, threaded=False):
        print(f"Соединение с {host}:{port}")
        cls.inst.connect((host, port))
        if threaded:
            threading.Thread(target=cls.inst.send_thread, daemon=True).start()
            threading.Thread(target=cls.inst.recv_thread, daemon=True).start()
        print("Соежинение установлено")

    @classmethod
    def add_recv_callback(cls, callback):
        cls.recv_callbacks.append(callback)

    @classmethod
    def update(cls):
        s._send()
        # recv = s._recv()
        if cls.recv_callbacks:
            for calb in cls.recv_callbacks:
                calb(recv)

    @classmethod
    def sleep(cls, sec):
        s.update()
        time.sleep(sec)

    @classmethod
    def _send(cls):
        com = b''
        while commands:
            com += commands.pop()
        if com:
            print("Send:", com)
            s.sendall(com)
        time.sleep(0.01)

    @classmethod
    def _recv(cls):
        recv = s.recv(1024)
        return recv

    @classmethod
    def send_thread(cls):
        s = cls.inst

        while 1:
            if not len(commands):
                continue
            try:
                command = commands.pop()
                print(f"Отправка команды: {command}")
                s.sendall(command)
            except socket.error as e:
                print(f"Ошибка сокета: {e}")
                return False

    @classmethod
    def recv_thread(cls):
        print("Recieving data")
        s = cls.inst
        while not cls.inst:
            pass
        while 1:
            recv = s.recv(1024)
            # print(list(recv))

    @classmethod
    def send_buf(cls, buf):
        commands.append(buf)


s = TCPSocket()
