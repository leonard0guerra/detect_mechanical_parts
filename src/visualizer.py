#!/usr/bin/env python3

import sys
import rospy
import cv2 as cv
from detector_node import DetectorNode
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Visualizer:

  def __init__(self, rec_video=False):
    self.__bridge = CvBridge()
    rospy.Subscriber(DetectorNode.TOPIC_IMAGE, Image, self.callback)

    self.__writer = None
    self.__rec_video = rec_video

    rospy.init_node('visualizer', anonymous=True)


  def callback(self,data):
    try:
      image = self.__bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as cve:
      rospy.logerr(str(cve))
      return
    
    cv.imshow('Detection', image)
    cv.waitKey(30)
   
def main(args):
  visualizer =  Visualizer()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.logerr('Shutting down')
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

