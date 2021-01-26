#!/usr/bin/env python3

import sys
import rospy
import message_filters
from detector import Detector
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectorNode:

    def __init__(self):
        rospy.init_node('detector_mechanical_parts', anonymous=True)
        self.node_name = rospy.get_name()

        yolo_cfg = rospy.get_param(f'/{self.node_name}/yolo_cfg')
        yolo_weights = rospy.get_param(f'/{self.node_name}/yolo_weights')
        obj_names = rospy.get_param(f'/{self.node_name}/obj_names')

        conf_threshold = float(rospy.get_param(f'/{self.node_name}/conf_threshold'))
        nms_threshold = float(rospy.get_param(f'/{self.node_name}/nms_threshold'))

        self.detector = Detector(yolo_cfg=yolo_cfg, \
                                 yolo_weights=yolo_weights,\
                                 obj_names=obj_names,\
                                 conf_threshold=conf_threshold,\
                                 nms_threshold=nms_threshold)
        
        self.__bridge = CvBridge()

        detections_image_topic = rospy.get_param(f'/{self.node_name}/detections_image_topic')
        image_source_topic = rospy.get_param(f'/{self.node_name}/image_source_topic')
        
        #buffer size = 2**24 | 480*640*3
        rospy.Subscriber(image_source_topic, Image, self.detect_image, queue_size=1, buff_size=2**24)    
        self.__publisher = rospy.Publisher(detections_image_topic, Image, queue_size=1)

        

    def detect_image(self, data):
        try:
            image = self.__bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as cve:
            rospy.logerr(str(cve))
            return
        
        outputs = self.detector.detect_objects(image)
        self.detector.process_image(outputs, image)


        try:
            detection_message = self.__bridge.cv2_to_imgmsg(image, "bgr8")
            self.__publisher.publish(detection_message)
        except CvBridgeError as cve:
            rospy.logerr(str(cve))
            return

def main(args):

    detector_node = DetectorNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr('Shutting down')

if __name__ == '__main__':
    main(sys.argv)