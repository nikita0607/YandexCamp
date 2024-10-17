import time
import numpy as np
import cv2

import colors

from ultralytics import YOLO

class Camera:
    cap: cv2.VideoCapture = None
    stream_url = None
    width = 0
    height = 0

    @classmethod
    def init_cam(cls, stream_url):
        cls.cap = cv2.VideoCapture(stream_url)
        cls.stream_url = stream_url

        if not cls.cap.isOpened():
            print("Не удается подключиться к видеопотоку.")
            exit()

    @classmethod
    def reset_cam(cls):
        cls.cap.release()
        cls.init_cam(cls.stream_url)

    @classmethod
    def get_frame(cls):
        ret, frame = cls.cap.read()
        cls.height, cls.width = frame.shape[:2]

        return frame


class Detector:
    model = YOLO('best-5n-30epoch.pt')

    @classmethod
    def is_object_visible(cls, obj):
        Camera.reset_cam()
        frame = Camera.get_frame()
        cords = cls.detect_yolo_obj(frame, obj)

        if len(cords):
            return True
        return False

    @classmethod
    def detect_cube(cls, img: np.ndarray, color_detection=False):
        if not color_detection:
            return cls.detect_yolo_obj(img, 3)
        else:
            return cls.detect_cube_color(img)

    @classmethod
    def detect_yolo_obj(cls, img: np.ndarray, obj=3):
        results = cls.model.predict(img)[0]
        obj_cords = []
        for i in range(len(results)):
            if int(results[i].boxes.cls[0].item()) == obj:
                obj_cords.append(results[i].boxes.xywh.numpy()[0])
        cv2.imshow("Camera", results.plot())
        cv2.waitKey(500)

        return obj_cords

    @classmethod
    def detect_cube_color(cls, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        ranges = colors.Ranges.red_range

        # Определяем диапазон красного цвета в HSV (для разных оттенков красного)
        # Красный в HSV представлен в двух диапазонах: низкий и высокий

        # Маскируем изображение для двух диапазонов красного
        mask = None
        for rng in ranges:
            mask_r = cv2.inRange(hsv, rng[0], rng[1])
            if mask is not None:
                mask = cv2.bitwise_or(mask, mask_r)
            else:
                mask = mask_r


        # Применение морфологической операции для улучшения маски (например, открытие)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Нахождение момента изображения, чтобы найти центр масс
        moments = cv2.moments(mask)

        # Если находим массу пикселей
        if moments["m00"] != 0:
            # Координаты центра масс
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            # cv2.circle(img, (cx, cy), 10, (255, 0, 0), 3)

            # cv2.imshow("Original Image", img)
            # cv2.imshow("Red Mask", mask)

            return (cx, cy),
        else:
            print("Красные пиксели не найдены.")
            return None


if __name__ == '__main__':
    Camera.init_cam('http://192.168.2.42:8080/?action=stream')
    time.sleep(0.1)
    while 1:
        frame = Camera.get_frame()
        Detector.detect_cube(frame)