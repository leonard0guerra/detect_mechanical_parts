# detect_mechanical_parts :construction:

ROS package to read a stream of images, detect mechanical parts of cars using YOLOv4 and view the detection results.
___
## Launch Requirements
* ROS Noetic 1.15.9
* Python 3.4+
* OpenCV 4.4.0+
* Download yolo-obj.weights and place it in the [yolo_model](yolo_model) folder

## Detection
```shell
~$ roslaunch detect_mechanical_parts detect_mechanical_parts.launch
```