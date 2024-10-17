import cv2
import numpy as np

# Параметры шахматной доски
CHESSBOARD_SIZE = (9, 6)  # Количество углов по горизонтали и вертикали
SQUARE_SIZE = 1.0  # Размер одной клетки шахматной доски в единицах

# Подготовка объектов для 3D точек шахматной доски
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Списки для хранения 3D точек в пространстве и 2D точек на изображениях
objpoints = []
imgpoints = []

# Открытие видео
cap = cv2.VideoCapture('video.avi')

# Считываем кадры для калибровки
first_gray = None
while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Сохраним первое изображение для определения размера
    if first_gray is None:
        first_gray = gray

    # Поиск углов на шахматной доске
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Рисование углов
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners, ret)
        cv2.imshow('Chessboard', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

# Проверяем, есть ли у нас данные для калибровки
if len(objpoints) > 0 and first_gray is not None:
    # Калибровка камеры
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, first_gray.shape[::-1], None, None)

    # Применение устранения дисторсии к видео
    cap = cv2.VideoCapture('video.avi')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Устранение дисторсии
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        # Обрезка изображения по ROI, если нужно
        x, y, w, h = roi
        undistorted_frame = undistorted_frame[y:y + h, x:x + w]

        cv2.imshow('Undistorted Video', undistorted_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
else:
    print("Не удалось найти углы на шахматной доске. Попробуйте другое видео или увеличьте количество кадров для калибровки.")