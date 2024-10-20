import time

import ultralytics
import cv2
from ultralytics import YOLO
import numpy as np
import math

from movement import Movement
from socket_conn import TCPSocket

ip_camera_url_right = "rtsp://Admin:rtf123@192.168.2.250/251:554/1/1"# Создаем объект VideoCapture для захвата видео с IP-камеры


class UpCamera:
    @staticmethod
    def get_frame():
        cap = cv2.VideoCapture(ip_camera_url_right)#video_capture = cv2.VideoCapture(ip_camera_url_right)
        ret = False
        while not ret:
            ret, frame = cap.read()
        cap.release()
        return frame


## код для А*
class AStarPath:
    def __init__(self, robot_radius, grid_size, x_obstacle, y_obstacle, show_animation=True):
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        self.create_obstacle_map(x_obstacle, y_obstacle)
        self.path = self.get_path()
        self.show_animation = show_animation

    class Node:
        def __init__(self, x, y, cost, path):
            self.x = x
            self.y = y
            self.cost = cost
            self.path = path

        def __str__(self):
            return str(self.x)+'+'+str(self.y)+','+str(self.cost)+','+str(self.path)

    @staticmethod
    def calc_heuristic(num1, num2):
        weight = 1.0
        return weight * math.sqrt((num1.x-num2.x)**2 + (num1.y-num2.y)**2)

    def calc_grid_position(self, idx, p):
        return idx * self.grid_size + p

    def calc_xy(self, position, min_position):
        return round((position-min_position) / self.grid_size)

    def calc_grid_idx(self, node):
        return (node.y-self.y_min) * self.x_width + (node.x-self.x_min)

    def check_validity(self, node):
        x_position = self.calc_grid_position(node.x, self.x_min)
        y_position = self.calc_grid_position(node.y, self.y_min)

        if x_position < self.x_min:
            return False
        elif y_position < self.y_min:
            return False
        elif x_position >= self.x_max:
            return False
        elif y_position >= self.y_max:
            return False
        if self.obstacle_pos[node.x][node.y]:
            return False
        return True

    def create_obstacle_map(self, x_obstacle, y_obstacle):
        self.x_min = round(min(x_obstacle))
        self.y_min = round(min(y_obstacle))
        self.x_max = round(max(x_obstacle))
        self.y_max = round(max(y_obstacle))
        self.x_width = round((self.x_max - self.x_min) / self.grid_size)
        self.y_width = round((self.y_max - self.y_min) / self.grid_size)
        self.obstacle_pos = [[False for i in range(self.y_width)] for i in range(self.x_width)]
        for idx_x in range(self.x_width):
            x = self.calc_grid_position(idx_x, self.x_min)
            for idx_y in range(self.y_width):
                y = self.calc_grid_position(idx_y, self.y_min)
                for idx_x_obstacle, idx_y_obstacle in zip(x_obstacle, y_obstacle):
                    d = math.sqrt((idx_x_obstacle - x)**2 + (idx_y_obstacle - y)**2)
                    if d <= self.robot_radius:
                        self.obstacle_pos[idx_x][idx_y] = True
                        break

    @staticmethod
    def get_path():
        path = [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

        return path

    def calc_final_path(self, end_node, record_closed):
        x_out_path, y_out_path = [self.calc_grid_position(end_node.x, self.x_min)], [self.calc_grid_position(end_node.y, self.y_min)]
        path = end_node.path
        while path != -1:
            n = record_closed[path]
            x_out_path.append(self.calc_grid_position(n.x, self.x_min))
            y_out_path.append(self.calc_grid_position(n.y, self.y_min))
            path = n.path

        return x_out_path, y_out_path

    def a_star_search(self, start_x, start_y, end_x, end_y):
        start_node = self.Node(self.calc_xy(start_x, self.x_min), self.calc_xy(start_y, self.y_min), 0.0, -1)
        end_node = self.Node(self.calc_xy(end_x, self.x_min), self.calc_xy(end_y, self.y_min), 0.0, -1)

        record_open, record_closed = dict(), dict()
        record_open[self.calc_grid_idx(start_node)] = start_node

        while True:
            if len(record_open) == 0:
                print('Check Record Validity')
                break

            total_cost = min(record_open, key=lambda x: record_open[x].cost + self.calc_heuristic(end_node, record_open[x]))
            cost_collection = record_open[total_cost]

            if cost_collection.x == end_node.x and cost_collection.y == end_node.y:
                print("Finished!")
                end_node.path = cost_collection.path
                end_node.cost = cost_collection.cost
                break

            del record_open[total_cost]
            record_closed[total_cost] = cost_collection

            for i, _ in enumerate(self.path):
                node = self.Node(cost_collection.x + self.path[i][0], cost_collection.y + self.path[i][1], cost_collection.cost + self.path[i][2], total_cost)
                idx_node = self.calc_grid_idx(node)

                if not self.check_validity(node):
                    continue

                if idx_node in record_closed:
                    continue

                if idx_node not in record_open:
                    record_open[idx_node] = node
                else:
                    if record_open[idx_node].cost > node.cost:
                        record_open[idx_node] = node

        x_out_path, y_out_path = self.calc_final_path(end_node, record_closed)

        return x_out_path, y_out_path

## основа

# выравниваем картинку, если она под наклоном
def rotate(img):
    def get_angle(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        if len(contours) == 0:
            return 0.0
        largestContour = contours[0]
        minAreaRect = cv2.minAreaRect(largestContour)

        angle = minAreaRect[-1]
        if angle < -45:
            angle = 90 + angle
        return -1.0 * angle

    angle = get_angle(img)
    (h, w) = img.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, - 1.0 * angle, 1.0)
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

# на вход подаётся картинка в самую первую секунду заезда
def build_map(image):
    # ищем объекты
    r = model(image)[0]
    boxes = r.boxes.xyxy.tolist()
    classes = r.boxes.cls.tolist()
    names = r.names

    # ищем преграды и прочее
    cube_boxes = []
    robot_boxes = []
    image_patched = image.copy()
    for box, cls in zip(boxes, classes):
        label = f"{names[int(cls)]}"
        x1, y1, x2, y2 = map(int, box)
        if label == 'red_base' or label == 'green_base' or label == 'basket':
            cv2.rectangle(image_patched, (x1, y1), (x2, y2), color=(255, 255, 255), thickness=-1)
        elif label == 'cube':
            cube_boxes.append(box)
            cv2.rectangle(image_patched, (x1, y1), (x2, y2), color=(255, 255, 255), thickness=-1)
        elif label == 'robot' or label == 'red_robot' or label == 'green_robot':
            robot_boxes.append(box)
            cv2.rectangle(image_patched, (x1, y1), (x2, y2), color=(255, 255, 255), thickness=-1)
        elif label == 'red_button' or label == 'pink_button':
            cv2.rectangle(image_patched, (x1, y1), (x2, y2), color=(0, 0, 0), thickness=-1)

    # трешхолдим и создаём сам лабиринт
    gray = cv2.cvtColor(image_patched, cv2.COLOR_BGR2GRAY)
    threshold_value = 90
    _, img_binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    img_opened = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, kernel_open)
    img_closed = cv2.morphologyEx(img_opened, cv2.MORPH_CLOSE, kernel_close)

    return img_closed, robot_boxes, cube_boxes

def cubes_preprocessing(cube_boxes, robot_boxes):
    # делаем так, что первый куб всегда тот, что находится левее
    cube_1 = cube_boxes[0]
    cube_2 = cube_boxes[1]
    cube_1, cube_2 = (cube_1, cube_2) if cube_1[0] < cube_2[0] else (cube_2, cube_1)

    center_1 = [cube_1[0] + (cube_1[2] - cube_1[0]) / 2, cube_1[1] + (cube_1[3] - cube_1[1]) / 2]
    center_2 = [cube_2[0] + (cube_2[2] - cube_2[0]) / 2, cube_2[1] + (cube_2[3] - cube_2[1]) / 2]
    distance = np.sqrt((center_1[0] - center_2[0])**2 + (center_1[1] - center_2[1])**2)

    # print(f'distnace = {distance}')
    cube_type = ''
    if distance >= 500:
        cube_type = 'outer_upper_left_lower_right'
    else:
        if center_1[0] < center_2[0] and center_1[1] > center_2[1]:  # куб с меньшим Х находится ниже
            cube_type = 'inner_lower_left_upper_right'
        else:
            cube_type = 'inner_upper_left_lower_right'

    # безопасные точки
    robot_radius = abs(robot_boxes[0][0] - robot_boxes[0][2]) / 2
    if cube_type == 'inner_lower_left_upper_right':
        center_1[0] += robot_radius
        center_1[1] -= robot_radius
        center_2[0] -= robot_radius
        center_2[1] += robot_radius

    elif cube_type == 'inner_upper_left_lower_right' or cube_type == 'outer_upper_left_lower_right':
        center_1[0] += robot_radius
        center_1[1] += robot_radius
        center_2[0] -= robot_radius
        center_2[1] -= robot_radius

    return center_1, center_2


def plan_path(map_image, robot, endpoint, new_width=320):
    new_height = int(new_width * image.shape[0] / image.shape[1])
    ratio_width = image.shape[1] / new_width
    ratio_height = image.shape[0] / new_height

    # до сжатия; изначальные координаты центра робота
    start_x = robot[0] + abs(robot[0] - robot[2]) / 2
    start_y = robot[1] + (robot[3] - robot[1]) / 2

    # после сжатия
    start_x = start_x / ratio_width
    start_y = (image.shape[0] - start_y) / ratio_height

    end_x = endpoint[0] / ratio_width
    end_y = (image.shape[0] - endpoint[1]) /  ratio_height

    grid_size = 4.0
    robot_radius = abs(robot[0] - robot[2]) / (2 * ratio_width)
    robot_radius = 7.8  # TODO: безопасный радиус для робота (зависит от силы сжатия поля)
    print(f'robot radius (with ratio) = {robot_radius}')

    field = cv2.resize(map_image, (new_width, new_height))
    (width, length) = field.shape
    x_obstacle, y_obstacle = [], []
    for i in range(width):
        for j in range(length):
            if field[i][j] <= 150:
                y_obstacle.append(new_height-i)
                x_obstacle.append(j)

    start = time.time()
    a_star = AStarPath(robot_radius, grid_size, x_obstacle, y_obstacle, False)
    x_out_path, y_out_path = a_star.a_star_search(start_x, start_y, end_x, end_y)
    print(f'time taken: {time.time() - start} s')

    # АППРОКСИМАЦИЯ
    new_path = []
    for i in range(len(x_out_path)):
        new_path.append((x_out_path[i], y_out_path[i]))
    new_path_array = np.array(new_path).astype(np.float32)

    start = time.time()
    path_len = cv2.arcLength(new_path_array, closed=False)
    epsilon = 0.015 * path_len
    approx = cv2.approxPolyDP(new_path_array, epsilon, closed=False)
    print(f'approx time: {time.time() - start} s')

    x_app = []
    y_app = []

    for i in range(approx.shape[0]):
        for j in range(approx.shape[1]):
            x_app.append(approx[i][j][0])
            y_app.append(approx[i][j][1])

    return x_app, y_app, np.array(x_app) * ratio_width, np.array(y_app) * ratio_height

# для случая после выравнивания относительно вертикальной оси
def angles(x_coords, y_coords):
    angles = []
    for i in range(1, len(x_coords) - 1):
        dx1 = x_coords[i] - x_coords[i - 1]
        dy1 = y_coords[i] - y_coords[i - 1]
        angle1 = np.arctan2(dy1, dx1)
        dx2 = x_coords[i + 1] - x_coords[i]
        dy2 = y_coords[i + 1] - y_coords[i]
        angle2 = np.arctan2(dy2, dx2)
        angle_difference = np.degrees(angle2 - angle1)
        angle_difference = (angle_difference + 180) % 360 - 180
        angles.append(angle_difference)
    return angles

def times(x_coords, y_coords, velocity):
    times = []
    for i in range(1, len(x_coords)):
        dx = x_coords[i] - x_coords[i - 1]
        dy = y_coords[i] - y_coords[i - 1]
        distance = np.sqrt(dx**2 + dy**2)
        time = distance / velocity
        times.append(time)
    return times


robot_color = 'green'  # TODO
model = YOLO("best-top-down-5n-100epoch.pt")

if __name__ == '__main__':
    TCPSocket.connect_to('192.168.2.42', 2001)
    TCPSocket.send_buf(b'\xff\x06\x02\x00\xff')
    Movement().update_servo()
    TCPSocket.update()

    frame = UpCamera.get_frame()
    cv2.imshow("AA", frame)
    cv2.waitKey(500)
    time.sleep(1)

    image = rotate(frame)

    map_image, robots, cubes = build_map(image)
    center_1, center_2 = cubes_preprocessing(cubes, robots)
    x, y, x_big, y_big = plan_path(map_image, robots[1], center_2, new_width=360)
    # print(x, y)



    angl = angles(x_big, y_big)
    # angles.insert(0, 0)
    tms = times(x_big, y_big, 60)

    for i in range(max(len(angl), len(tms))):
        if (i < len(tms)):
            Movement.move_sync(1)
            TCPSocket.sleep(tms[i].item())
            Movement.move_sync(0)
        if (i < len(angl)):
            print(angl[i].item())
            Movement.rotate(angl[i].item())
            TCPSocket.sleep(angl[i].item())

    print(f'angles: {angl}')
    print(f'times: {tms}')



