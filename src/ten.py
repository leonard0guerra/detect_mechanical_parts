#!/usr/bin/env python

## Simple ten that published std_msgs/Strings messages
## to the 'filtered' topic

import rospy
from std_msgs.msg import String

class Ten:

    def __init__(self):
        rospy.Subscriber('filtered', String, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    

def main():
    ten = Ten()


    rospy.init_node('ten', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()