import cv2
import numpy as np

import colors




class ColorDetector:
    def __init__(self, stream_url):
        self.cap = cv2.VideoCapture(stream_url)
        self.stream_url = stream_url

        if not self.cap.isOpened():
            print("Не удается подключиться к видеопотоку.")
            exit()

    def reset(self):
        self.cap.release()
        self.cap = cv2.VideoCapture(self.stream_url)

    def find_center(self, img, ranges):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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

            return cx, cy
        else:
            print("Красные пиксели не найдены.")
            return None

if __name__ == '__main__':
    det = ColorDetector(stream_url = 'http://192.168.2.42:8080/?action=stream')

    while True:
        # Чтение кадра из видеопотока
        ret, frame = det.cap.read()
        sx, sy = frame.shape[:2]

        c = det.find_center(frame, ranges=[(colors.Ranges.lower_orange, colors.Ranges.upper_orange)])
        print(c)
        if c:
            cv2.waitKey(1)

    # Завершение работы и освобождение ресурсов
    cap.release()
    cv2.destroyAllWindows()