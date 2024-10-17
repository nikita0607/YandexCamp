import numpy as np

class Ranges:
    lower_orange = np.array([7, 40, 56])
    upper_orange = np.array([82, 176, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    orange_range = (lower_orange, upper_orange),
    red_range = ((lower_red1, upper_red1), (lower_red2, upper_red2))