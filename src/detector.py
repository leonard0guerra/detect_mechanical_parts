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

    CONF_THRESHOLD = 0.3
    NMS_THRESHOLD = 0.4


    def __init__(self):
        self.__init_node()
        self.__init_colors_for_classes()
        self.__init_network()

    def __init_node(self):
        self.__bridge = CvBridge()

        rospy.Subscriber(Detector.TOPIC_DATA, Image, self.callback)    
        self.__publisher = rospy.Publisher(Detector.TOPIC_IMAGE, Image, queue_size=50)
    
    def __init_colors_for_classes(self):
        self.labels = []
        with open(Detector.OBJ_NAMES, "r") as f:
            self.labels = [cname.strip() for cname in f.readlines()]
        
        self.bbox_colors = np.random.uniform(low=0, high=255, size=(len(self.labels), 3))
        
    
    def __init_network(self):
        self.net = cv.dnn.readNetFromDarknet(Detector.YOLO_CFG, darknetModel=Detector.YOLO_WEIGHTS)

        layer_names = self.net.getLayerNames()
        self.__output_layer = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def detect_objects(self, image):
        blob = cv.dnn.blobFromImage(image, scalefactor=1/255, size=(416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.__output_layer)
        return outputs
    
    def process_frame(self, outputs, image):
        boxes, confidences, classIDs = [], [], []

        H, W = image.shape[:2]
                                    
        for out in outputs:
            for detection in out:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                                        
                if confidence > Detector.CONF_THRESHOLD:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)
                                        
        idxs = cv.dnn.NMSBoxes(boxes, confidences, score_threshold=Detector.CONF_THRESHOLD, nms_threshold=Detector.NMS_THRESHOLD)
                                        
        if len(idxs)>0:
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                                        
                clr = [int(c) for c in self.bbox_colors[classIDs[i]]]
                                        
                cv.rectangle(image, (x, y), (x+w, y+h), clr, 2)
                cv.putText(image, "{}: {:.4f}".format(self.labels[classIDs[i]], confidences[i]), (x, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.5, clr, 2)



    def callback(self, data):
        try:
            image = self.__bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as cve:
            rospy.logerr(str(cve))
            return
        
        outputs = self.detect_objects(image)
        self.process_frame(outputs, image)


        try:
            detection_message = self.__bridge.cv2_to_imgmsg(image, "bgr8")
            self.__publisher.publish(detection_message)
        except CvBridgeError as cve:
            rospy.logerr(str(cve))
            return

def main(args):
    detector = Detector()
    rospy.init_node('detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr('Shutting down')
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)