from Detection2D import Detection2D
from numpy import ndarray
class Detection2DArray: 
    def __init__(self, img: ndarray, detections: list[Detection2D], header):
        self.img = img
        self.detections = detections
        self.header = header
