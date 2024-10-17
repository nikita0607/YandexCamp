from socket_conn import TCPSocket
from detector import *
from movement import Movement, DefaultAngles


class Skills:
    @staticmethod
    def move_to_obj(cord_num=0, move_val=0.02, dv=20, max_iter=6, p=0.08, obj=3, sonar_stop=False):
        cdv = dv * 2
        iter_num = 0
        TCPSocket.sleep(0.2)
        Camera.reset_cam()

        def get_cdv():
            Movement.set_speed(40)
            frame = None
            Camera.reset_cam()
            while frame is None:
                frame = Camera.get_frame()

            sv = frame.shape[:2][1 - cord_num]
            cv = sv // 2

            cords = Detector.detect_yolo_obj(frame, obj=obj)

            if not len(cords):
                return False

            cords = cords[0]
            cdv =  cv - cords[cord_num]
            if cdv > 0:
                Movement.move(1 + 2 * (1 - cord_num), 0.1*cdv/100)
            else:
                Movement.move(2 + 2 * (1 - cord_num), 0.1*cdv/100)
            TCPSocket.update()

            return cdv

        cdv = abs(get_cdv())
        while cdv > dv:
            while cdv > dv:
                cdv = abs(get_cdv())
                TCPSocket.sleep(0.2)
            Movement.move_sync(0)
            TCPSocket.sleep(0.5)

            cdv = abs(get_cdv())



    @staticmethod
    def search_obj_pipe(obj):
        def _get(i, reversed):
            Movement.cam_rotate = DefaultAngles.CAM_CUBE_GRAB_ROTATE + i
            Movement.update_servo()
            TCPSocket.sleep(0.3)
            Camera.reset_cam()

            frame = Camera.get_frame()
            cords = Detector.detect_yolo_obj(frame, obj=obj)

            if len(cords):
                print("Cords; ", cords)
                return sorted(cords, key=lambda x: x[0], reverse=reversed)[0]

        for i in range(0, 80, 13):
            cr = _get(i, True)
            if cr is not None:
                return cr

            cr = _get(-i, False)
            if cr is not None:
                return cr

        return

    @staticmethod
    def rotate_to_searched(cords):
        k1 = 1
        k2 = 1
        k3 = 3.36/360

        rang = Movement.cam_rotate - DefaultAngles.CAM_CUBE_GRAB_ROTATE
        rscreen = -(cords[0] - (Camera.width // 2)) // (Camera.width // 2) * 25

        t = (rscreen*k1 + rang*k2) * k3
        if t > 0:
            Movement.move(3, abs(t))
        else:
            Movement.move(4, abs(t))

        Movement.move(0, abs(t))
        TCPSocket.sleep(abs(t))

        Movement.cam_rotate = DefaultAngles.CAM_CUBE_GRAB_ROTATE
        Movement.set_speed(40)
        Movement.update_servo()
        TCPSocket.update()

    @staticmethod
    def move_to_obj_pipe(obj=3):
        Movement.cam_rotate = DefaultAngles.CAM_CUBE_GRAB_ROTATE
        Movement.cam_pitch = DefaultAngles.CAM_CUBE_GRAB_PITCH
        Skills.move_to_obj_pipe_stage(obj)

    @staticmethod
    def move_to_obj_pipe_stage(obj=3):
        Movement.set_speed(5)
        TCPSocket.update()

        while 1:
            while not Detector.is_object_visible(obj):
                crd = Skills.search_obj_pipe(obj)
                TCPSocket.sleep(0.5)
                if crd is not None:
                    Skills.rotate_to_searched(crd)
                else:
                    return 0


            a = Skills.move_to_obj(dv=40, move_val=0.03, obj=obj,)
            b = Skills.move_to_obj(1, dv=30, move_val=0.05, obj=obj, p=0.1)
            a = Skills.move_to_obj(dv=40, move_val=0.03, obj=obj, p=0.08)

            if Detector.is_object_visible(3):
                return 1


class Grabbing:

    @staticmethod
    def grab_obj(obj):
        if obj == 3:
            Grabbing.grab_cube()
            return
        raise ValueError(f"Tried to grab object {obj}!")

    @staticmethod
    def grab_cube():
        Movement.set_speed(50)
        Movement.first_angle = 40
        Movement.hand_angle = 40
        Movement.update_servo()
        TCPSocket.sleep(0.2)
        Movement.move(1, 1)
        TCPSocket.sleep(1.3)
        Movement.hand_angle += 40
        Movement.update_servo()
        TCPSocket.sleep(0.5)
        Movement.move(2, 2)
        Movement.first_angle = 180
        Movement.update_servo()
        TCPSocket.update()


    @staticmethod
    def grab_sphere():
        Movement.first_angle = 160
        Movement.second_angle = 40
        Movement.hand_angle = 50
        Movement.update_servo()
        time.sleep(1)
        Movement.move(1, .5)
        time.sleep(.5)
        Movement.move(0, 0)
        time.sleep(1)
        Movement.hand_angle += 110
        Movement.update_servo()
        time.sleep(0.5)
        Movement.second_angle += 100
        Movement.update_servo()
