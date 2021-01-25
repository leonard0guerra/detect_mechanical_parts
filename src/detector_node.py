#!/usr/bin/env python3

import sys
import rospy
import message_filters
from detector import Detector
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectorNode:

    # ROS Topic 
    TOPIC_DATA = '/device_0/sensor_1/Color_0/image/data'
    TOPIC_IMAGE = '/detect_mechanical_parts/image_detection'

    def __init__(self):
        self.detector = Detector()

    def init_node(self, topic_data:str, topic_image_detection:str):
        self.__bridge = CvBridge()
        
        #buffer size = 2**24 | 480*640*3
        rospy.Subscriber(topic_data, Image, self.detect_image, queue_size=1, buff_size=2**24)    
        self.__publisher = rospy.Publisher(topic_image_detection, Image, queue_size=1)

        rospy.init_node('detector_mechanical_parts', anonymous=True)

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
    detector_node.init_node(DetectorNode.TOPIC_DATA, DetectorNode.TOPIC_IMAGE)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr('Shutting down')

if __name__ == '__main__':
    main(sys.argv)