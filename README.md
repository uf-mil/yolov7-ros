# ROS package for official YOLOv7

This repo contains a ROS noetic package for the official YOLOv7. It wraps the 
[official implementation](https://github.com/WongKinYiu/yolov7) into a ROS node (so most credit 
goes to the YOLOv7 creators).

### Note
There are currently two YOLOv7 variants out there. This repo contains the 
implementation from the paper [YOLOv7: Trainable bag-of-freebies sets new state-of-the-art for real-time object detectors](https://arxiv.org/abs/2207.02696).

## Requirements & Getting Started

Following ROS packages are required:
- [vision_msgs](http://wiki.ros.org/vision_msgs)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)

First, clone the repo into your catkin workspace and build the package:
```
git clone https://github.com/lukazso/yolov7-ros.git ~/catkin_ws/src/
cd ~/catkin_ws
catkin build yolov7_ros
```

The Python requirements are listed in the `requirements.txt`. You can simply 
install them as
```
pip install -r requirements.txt
```

Download the YOLOv7 weights from the [official repository](https://github.com/WongKinYiu/yolov7).

The package has been tested under Ubuntu 20.04 and Python 3.8.10.

## Usage
Before you launch the node, adjust the parameters in the 
[launch file](launch/yolov7.launch). For example, you need to set the path to your 
YOLOv7 weights and the image topic to which this node should listen to. The launch 
file also contains a description for each parameter.

```
roslaunch yolov7_ros yolov7.launch
```

Each time a new image is received it is then fed into YOLOv7.

### Notes
- The detections are published using the [vision_msgs/Detection2DArray](http://docs.ros.org/en/api/vision_msgs/html/msg/Detection2DArray.html) message type.
- The detections will be published under `/yolov7/out_topic` defined in the [launch file](launch/yolov7.launch).
- If you set the `visualize` parameter to `true`, the detections will be drawn into 
  the image, which is then published under `/yolov7/out_topic/visualization`.

### MIL Training (TODO: Make this setup process more automated.)
- Go to your labelbox project and export the labels.
- Outside of the repository, clone this repo: git@github.com:ultralytics/JSON2YOLO.git.
- Use it to convert your labelbox export into a directory containing the images and corresponding labels.
- This can be accomplished by moving your labelbox export within this repo, change the filename within ```labelbox_json2yolo.py```, run this python script. (This can take a while if your dataset is large.)
- This will create a directory with the same name as your .json file you exported.
- Within this new directory, you will see an images folder and a labels folder.
- Create a train and val folder within both the images and labels folders, and place a certain number of files within each folder. (Make sure the files correspond with each other. Ex. if image1.png is in train, ensure image1.txt is in labels.)
- Within the file set as `data.yaml` in the [launch file](launch/yolov7.launch), make the train go to the path ```<zip file name>/images/train```, and make the val go to path ```<zip file name>/images/val```
- Move this the folder containing the images and labels to the src directory of the yolov7_ros package.
- Move the .yaml within the folder you just moved into yolov7-ros/src/data.
- Confirm in `cfg/training/yolov7.yaml` the number of possible classes.
- Run the following command, feel free to change any parameters as you see fit. You can see what params you can change within train.py.
```python3 train.py --workers 8 --device 0 --batch-size 4 --data data/buoys.yaml --img 640 360 --cfg cfg/training/yolov7.yaml --weights 'yolov7_training.pt' --name yolov7 --hyp data/hyp.scratch.p5.yaml```
- The resulting model (weights) will appear in "yolov7-ros/src/runs/train/yolov71i/weights/best.pt" where that i is the number of iteration. The latest training results should have the bigger i, but double check the date just in case.

### MIL Single Image Testing
```python3 detect.py --weights runs/train/<path to best.pt> --conf 0.7 --img-size 640 --source buoys/images/<path to any image>```
