
from socket_conn import TCPSocket
from detector import *
from movement import Movement, DefaultAngles
# для ПИД
dx_pred = 0
arr_of_div = []

Ti = 20
Kp = 1*1.109
Kd = Kp*Ti/16
Ki = Kp/Ti
#

class Skills:
    cam = None
    @staticmethod
    def move_to_obj(cord_num=0, move_val=0.02, adx=20, ady=20, max_iter=0, p=0.008, obj=3, sonar_stop=False):
        Movement.set_speed(8)
        cdx = adx*2
        cdy = ady*2

        while abs(cdx) > adx and abs(cdy) > ady:
            frame = None
            while frame is None:
                frame = Camera.get_frame()

            sy, sx = frame.shape[:2]
            cy, cx = sy//2, sx//2

            cords = Detector.detect_yolo_obj(frame, obj=obj)

            if not len(cords):
                return False

            cords = cords[0]  # [[1, 2, 3], []]
            cdx = cx - cords[0]
            cdy = cy - cords[1]
            print(cdy)

            napr = 1 if cdy > 0 else 2

            # Вряд ли, но вдруг поможет
            global dx_pred

            ws = 0.092

            linear = abs(cdy)*0.15

            if len(arr_of_div)>Ti:
                arr_of_div.pop(0)

            angular = (abs(cdx)-adx)*Kp + (cdx-dx_pred) * Kd  + sum(arr_of_div)*Ki
            if cdx < 0:
                angular = -angular

            dx_pred = cdx
            # потом не забыть dx_pred приравнять к 0, после захвата куба

            vl = linear - angular * ws
            vr = linear + angular * ws

            print("SPID:", vl, vr)
            if vl > 90:
                vl = 90
            if vl < 10:
                vl = 10
            if vr > 90:
                vr = 90
            if vr < 10:
                vr = 10

            if napr == 2:
                vr, vl = vl, vr

            Movement.set_speed_l(int(vl))
            Movement.set_speed_r(int(vr))


            TCPSocket.sleep(0.1)
            Movement.move_sync(napr)
            TCPSocket.sleep(0.1)
        Movement.move_sync(0)
        TCPSocket.sleep(0.1)


    @staticmethod
    def search_obj_pipe(obj):
        Movement.cam_pitch = DefaultAngles.CAM_SEARCH_PITCH
        Movement.update_servo()
        TCPSocket.update()

        def _get(i, reversed):
            Movement.cam_rotate = DefaultAngles.CAM_CUBE_GRAB_ROTATE + i
            Movement.update_servo()
            TCPSocket.sleep(0.5)
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

        Movement.update_servo()
        TCPSocket.update()

        return

    @staticmethod
    def rotate_to_searched(cords):

        Movement.set_speed(40)
        k1 = 1
        k2 = 1
        k3 = 3.36/360

        rang = Movement.cam_rotate - DefaultAngles.CAM_CUBE_GRAB_ROTATE
        rscreen = -(cords[0] - (Camera.width // 2)) // (Camera.width // 2) * 25

        t = (rscreen*k1 + rang*k2) * k3
        print("RANG : ", rang)
        Movement.rotate(-rang)

        TCPSocket.sleep(1)

    @staticmethod
    def setup_move_pipeline(obj, start_cam_pose=None):
        if start_cam_pose:
            Movement.cam_pitch = start_cam_pose[0]
            Movement.cam_rotate = start_cam_pose[1]

            Movement.update_servo()
            TCPSocket.update()

            return

        Movement.cam_rotate = DefaultAngles.CAM_CUBE_GRAB_ROTATE
        Movement.cam_pitch = DefaultAngles.CAM_CUBE_GRAB_PITCH
        if obj == 1:
            Movement.cam_pitch = DefaultAngles.CAM_DROP_BASKET_PITCH
        elif obj == 4:
            Movement.cam_pitch = DefaultAngles.CAM_BUTTON_PRESS_PITCH
            Movement.cam_rotate = DefaultAngles.CAM_BUTTON_PRESS_ROTATE
        elif obj == 0:
            Movement.cam_pitch = DefaultAngles.CAM_BALL_GRAB_PITCH
            Movement.cam_rotate = DefaultAngles.CAM_BALL_GRAB_ROTATE

        Movement.update_servo()
        TCPSocket.update()

    @staticmethod
    def move_to_obj_pipe(obj=3, adx=40, ady=40, max_iter=0, start_cam_pose=None):
        Skills.setup_move_pipeline(obj, start_cam_pose=start_cam_pose)
        Skills.move_to_obj_pipe_stage(obj, adx=adx, ady=ady, max_iter=max_iter, start_cam_pose=start_cam_pose)

    @staticmethod
    def move_to_obj_pipe_stage(obj=3, adx=40, ady=40, max_iter=0, should_ret=False, start_cam_pose=None):
        TCPSocket.update()

        while 1:
            while not Detector.is_object_visible(obj):
                crd = Skills.search_obj_pipe(obj)
                TCPSocket.sleep(0.5)
                if crd is not None:
                    Skills.rotate_to_searched(crd)
                    if should_ret:
                        Skills.setup_move_pipeline(obj)
                        return
                    Skills.move_to_obj_pipe_stage(obj, adx, ady, should_ret=True, start_cam_pose=start_cam_pose)
                    Skills.setup_move_pipeline(obj, start_cam_pose)
                    TCPSocket.sleep(1)
                else:
                    return 0

            a = Skills.move_to_obj(adx=adx, ady=ady, move_val=0.03, obj=obj, max_iter=max_iter)
            b = Skills.move_to_obj(1, adx=adx, ady=ady, move_val=0.05, obj=obj, p=0.03, max_iter=max_iter)
            a = Skills.move_to_obj(adx=adx, ady=ady, move_val=0.03, obj=obj, p=0.008, max_iter=max_iter)
            b = Skills.move_to_obj(1, adx=adx, ady=ady, move_val=0.05, obj=obj, p=0.02, max_iter=max_iter)

            if Detector.is_object_visible(obj):
                Skills.setup_move_pipeline(obj, start_cam_pose)
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
        Movement.hand_angle += 30
        Movement.update_servo()
        TCPSocket.sleep(0.5)
        Movement.move(2, 1)
        Movement.first_angle = 180
        Movement.update_servo()
        TCPSocket.update()


    @staticmethod
    def drop_to_basket():
        Movement.first_angle = 100
        Movement.update_servo()
        TCPSocket.sleep(1)
        Movement.hand_angle = 20
        Movement.update_servo()
        TCPSocket.update()

    @staticmethod
    def press_button():
        Movement.first_angle = 100
        Movement.second_angle = 124
        Movement.update_servo()
        TCPSocket.sleep(0.7)
        Movement.first_angle = 180
        Movement.second_angle = 180

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
