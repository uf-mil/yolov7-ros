import rospy
import torch
from numpy import ndarray
from utils.Detection2DArray import Detection2DArray
from utils.Detection2D import Detection2D 
from utils.BoundingBox2D import BoundingBox2D
from utils.ObjectHypothesisWithPose import ObjectHypothesisWithPose

# todo: update to classes
class DetectionServer:
    def __init__(self): 
        self.detection_array = Detection2DArray()
        self.detection_history = []
        self.number_of_detections = 0
        self.objects_found = False

# todo: find our if I still need this
    def create_time_stamp():
        h = Header()
        h.stamp = rospy.Time.now()
        return h


    def update_detections(image: ndarray, detections: torch.Tensor) -> Detection2DArray:
        """
        :param detections: torch tensor of shape [num_boxes, 6] where each element is
            [x1, y1, x2, y2, confidence, class_id]
        :returns: detections as a ros message of type Detection2DArray
        """
        self.detection_array = Detection2DArray()

        # todo: add source image https://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection2D.html

        # header
        time_stamp = create_time_stamp()
        self.detection_array.header = time_stamp
        self.detection_array.img = image
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection.tolist()
            single_detection_msg = Detection2D()
            single_detection_msg.header = time_stamp


            # bbox
            bbox = BoundingBox2D()
            w = int(round(x2 - x1))
            h = int(round(y2 - y1))
            cx = int(round(x1 + w / 2))
            cy = int(round(y1 + h / 2))
            bbox.size_x = w
            bbox.size_y = h
            bbox.center_x = cx
            bbox.center_y = cy

            single_detection_msg.bbox = bbox

            # class id & confidence
            obj_hyp = ObjectHypothesisWithPose()
            obj_hyp.id = int(cls)
            obj_hyp.score = conf
            single_detection_msg.results = [obj_hyp]

            self.detection_array.detections.append(single_detection_msg)
        self.detection_history.append(self.detection_array)
        return self.detection_array

    def get_detections(): 
        if(len(self.detection_array)==0:
           raise Exception("Detections were requested but no objects were found")
        else: 
           return self.detection_array

    def get_number_of_objects_detected():
        return len(self.detection_array)

    def get_detection_history():
           return self.detection_history

    def erase_detection_history():
           self.detection_history = []


