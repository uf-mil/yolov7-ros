from typing import List

import cv2
import numpy as np

def get_random_color(seed):
    gen = np.random.default_rng(seed)
    color = tuple(gen.choice(range(256), size=3))
    color = tuple([int(c) for c in color])
    return color


def draw_detections(
    img: np.array, bboxes: List[List[int]], classes: List[int], class_names: List[str]
):
    for bbox, cls in zip(bboxes, classes):
        x1, y1, x2, y2 = bbox

        text = class_names[cls]
        color, background_color = (255, 255, 255), (0, 0, 0)
        # Get color and background color for label name
        if "red" in text:
            background_color = (255, 0, 0)
            color = (0, 0, 0)
            print("red")
        elif "orange" in text:
            background_color = (255, 127, 0)
            color = (0, 0, 0)
        elif "yellow" in text:
            background_color = (255, 255, 0)
            color = (0, 0, 0)
        elif "green" in text:
            background_color = (0, 255, 0)
            color = (0, 0, 0)
        elif "blue" in text:
            background_color = (0, 0, 255)
            color = (255, 255, 255)
        elif "purple" in text:
            background_color = (148, 0, 211)
            color = (0, 0, 0)
        elif "white" in text:
            background_color = (199, 199, 199)
            color = (0, 0, 0)
        elif "black" in text:
            background_color = (0, 0, 0)
            color = (255, 255, 255)
        else:
            background_color = get_random_color(int(cls))
            color = (0, 0, 0)

        # Create label and background box for label
        img = cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), background_color, 3)
        x_text = int(x1)
        y_text = max(15, int(y1 - 10))
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_COMPLEX, 0.5, 1)
        text_w, text_h = text_size
        cv2.rectangle(
            img,
            (x_text - 10, y_text - 15),
            (x_text + text_w + 10, y_text + text_h - 5),
            background_color,
            -1,
        )
        img = cv2.putText(
            img,
            text,
            (x_text, y_text),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    return img
