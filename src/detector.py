#!/usr/bin/env python3

import sys
import rospy
import cv2 as cv
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Detector:

    # TODO: use relative path
    YOLO_CFG = '/home/spark/catkin_ws/src/detect_mechanical_parts/yolo_model/yolo-obj.cfg'
    YOLO_WEIGHTS = '/home/spark/catkin_ws/src/detect_mechanical_parts/yolo_model/yolo-obj_last.weights'
    OBJ_NAMES = '/home/spark/catkin_ws/src/detect_mechanical_parts/yolo_model/obj.names'

    # ROS Topic 
    TOPIC_DATA = '/device_0/sensor_1/Color_0/image/data'
    TOPIC_IMAGE = '/detect_mechanical_parts/image'


    def __init__(self):
        self.__init_node()
        self.__init_colors_for_classes()
        self.__init_network()

    def __init_node(self):
        self.__bridge = CvBridge()

        rospy.Subscriber(Detector.TOPIC_DATA, Image, self.callback)    
        self.__publisher = rospy.Publisher(Detector.TOPIC_IMAGE, Image, queue_size=10)
    
    def __init_colors_for_classes(self):
        labels = []
        with open(Detector.OBJ_NAMES, "r") as f:
            labels = [cname.strip() for cname in f.readlines()]
        
        self.bbox_colors = np.random.uniform(low=0, high=255, size=(len(labels), 3))
        
    
    def __init_network(self):
        self.net = cv.dnn.readNetFromDarknet(Detector.YOLO_CFG, darknetModel=Detector.YOLO_WEIGHTS)

        layer_names = self.net.getLayerNames()
        self.__output_layer = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def detect_objects(self, image):
        blob = cv.dnn.blobFromImage(image, scalefactor=1 / 255, size=(416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.__output_layer)
        return outputs

    def callback(self, data):
        try:
            image = self.__bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as cve:
            rospy.logerr(str(cve))
            return

def main(args):
    detector = Detector()
    rospy.init_node('detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)