import numpy as np
import cv2
from typing import List

'''
CLASSES = {
        0: "yellow_cylinder",
        1: "black_round",
        2: "white_cylinder",
        3: "black_cylinder",
        4: "red_cylinder",
        5: "green_round",
        6: "white_round",
        7: "orange_round"
    }
'''
CLASSES = {
        0: "mb_marker_buoy_red",
        1: "mb_marker_buoy_green",
        2: "mb_marker_buoy_black",
        3: "mb_marker_buoy_white",
        4: "mb_round_buoy_black",
        5: "mb_round_buoy_orange"
    }

def get_random_color(seed):
    gen = np.random.default_rng(seed)
    color = tuple(gen.choice(range(256), size=3))
    color = tuple([int(c) for c in color])
    return color


def draw_detections(img: np.array, bboxes: List[List[int]], classes: List[int]):
    for bbox, cls in zip(bboxes, classes):
        x1, y1, x2, y2 = bbox

        color = get_random_color(int(cls))
        img = cv2.rectangle(
            img, (int(x1), int(y1)), (int(x2), int(y2)), color, 3
        )
        x_text = int(x1)
        y_text = max(15, int(y1 - 10))
        img = cv2.putText(
            img, CLASSES[cls], (x_text, y_text), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, color, 1, cv2.LINE_AA
        )

    return img
