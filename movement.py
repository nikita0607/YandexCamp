from socket_conn import TCPSocket as Tcp, TCPSocket


class DefaultAngles:
    CAM_ROTATE_MIN = 50
    CAM_ROTATE_MAX = 160

    CAM_CUBE_GRAB_ROTATE = 93
    CAM_CUBE_GRAB_PITCH = 31

    CAM_BUTTON_PRESS_PITCH = 39
    CAM_BUTTON_PRESS_ROTATE = 104

    CAM_BALL_GRAB_ROTATE = 94
    CAM_BALL_GRAB_PITCH = 30

    CAM_DROP_BASKET_PITCH = 22

    CAM_SEARCH_PITCH = 40


ROTATE_TIME = 4.15


class Movement:
    first_angle = 150
    second_angle = 180
    rotate_angle = 95
    hand_angle = 60

    cam_pitch = DefaultAngles.CAM_CUBE_GRAB_PITCH
    cam_rotate = DefaultAngles.CAM_CUBE_GRAB_ROTATE

    @staticmethod
    def set_speed(speed):
        Movement.set_speed_l(speed)
        Movement.set_speed_r(speed)

    @staticmethod
    def set_speed_l(speed):
        command = b'\xff\x02\x01' + speed.to_bytes() + b'\xff'
        Tcp.send_buf(command)

    @staticmethod
    def rotate(angle):
        time = abs(angle)/360*ROTATE_TIME
        if angle > 0:
            Movement.move(4, time)
        else:
            Movement.move(3, time)
        TCPSocket.sleep(time+0.1)

    @staticmethod
    def set_speed_r(speed):
        command = b'\xff\x02\x02' + (speed).to_bytes() + b'\xff'
        Tcp.send_buf(command)

    @staticmethod
    def move(com: int, timing: float):
        command = b'\xff\x09' + com.to_bytes() + int(timing*20).to_bytes() + b'\xff'
        Tcp.send_buf(command)

    @staticmethod
    def move_sync(com: int):
        print("MOVE")
        command = b'\xff\x00' + com.to_bytes() + b'\x00\xff'
        Tcp.send_buf(command)

    @classmethod
    def move_first_servo(cls, com):
        cls.first_angle += com
        cls.first_angle = min(max(0, cls.first_angle), 180)

        command = b'\xff\x01\x01' + cls.first_angle.to_bytes() + b'\xff'

        Tcp.send_buf(command)

    @classmethod
    def move_second_servo(cls, com):
        cls.second_angle += com
        cls.second_angle = min(max(0, cls.second_angle), 180)

        command = b'\xff\x01\x02' + cls.second_angle.to_bytes() + b'\xff'

        Tcp.send_buf(command)

    @classmethod
    def rotate_servo(cls, com):
        cls.rotate_angle += com
        cls.rotate_angle = min(max(0, cls.rotate_angle), 180)

        command = b'\xff\x01\x03' + cls.rotate_angle.to_bytes() + b'\xff'
        Tcp.send_buf(command)

    @classmethod
    def hand_servo(cls, com):
        cls.hand_angle += com
        cls.hand_angle = min(max(0, cls.hand_angle), 180)

        command = b'\xff\x01\x04' + cls.hand_angle.to_bytes() + b'\xff'

        Tcp.send_buf(command)

    @classmethod
    def rotate_cam(cls, com):
        cls.cam_rotate += com
        cls.cam_rotate = min(max(0, cls.cam_rotate), 180)

        command = b'\xff\x01\x07' + cls.cam_rotate.to_bytes() + b'\xff'

        Tcp.send_buf(command)

    @classmethod
    def pitch_cam(cls, com):
        cls.cam_pitch += com
        cls.cam_pitch = min(max(0, cls.cam_pitch), 180)

        command = b'\xff\x01\x08' + cls.cam_pitch.to_bytes() + b'\xff'

        Tcp.send_buf(command)
        Tcp.update()

    @classmethod
    def update_servo(cls):
        print(cls.second_angle)
        cls.move_first_servo(0)
        cls.move_second_servo(0)
        cls.hand_servo(0)
        cls.rotate_cam(0)
        cls.rotate_servo(0)
        cls.pitch_cam(0)

    #
    # def _update_thr(self):
    #     print("RUN")
    #     while True:
    #         self.update_servo()
    #         self.move(self.move_dest)
    #         time.sleep(0.01)

if __name__ == '__main__':
    Tcp.connect_to('192.168.2.42', 2001)
    Tcp.send_buf(b'\xff\x06\x01\x00\xff')
    Movement().update_servo()
    Tcp.update()
