from ObjectHypothesisWithPose import ObjectHypothesisWithPose
from BoundingBox1D import BoundingBox2D

class Detection2D:
    def __init__(self, results: ObjectHypothesisWithPose, bbox: BoundingBox2D, header):
        self.results = results
        self.bbox = bbox
        self.header = header
