#!/usr/bin/env python3

import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Visualizer:
  
  TOPIC_IMAGE = '/detect_mechanical_parts/image'


  def __init__(self, rec_video=False):
    self.__bridge = CvBridge()
    rospy.Subscriber(Visualizer.TOPIC_IMAGE, Image, self.callback)
    self.__writer = None
    self.__rec_video = rec_video


  def callback(self,data):
    try:
      image = self.__bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as cve:
      rospy.logerr(str(cve))
      return 
      
    H, W = image.shape[:2]

    cv.imshow('Detection', image)

    if self.__rec_video:
      if self.__writer is None:
        fourcc = cv.VideoWriter_fourcc(*"MJPG")
        self.__writer = cv.VideoWriter("result.avi", fourcc, 30, (W, H), True)
                                    
      self.__writer.write(image)

    cv.waitKey(30)
   
def main(args):
  visualizer = Visualizer(rec_video=True)
  rospy.init_node('visualizer', anonymous=True)
    
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.logerr('Shutting down')
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

