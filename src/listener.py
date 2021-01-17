#!/usr/bin/env python

## Simple talker that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

class Listener:

    def __init__(self):

        self.pub = rospy.Publisher('filtered', String, queue_size=5)
        self.count_msg = 0

        rospy.Subscriber('chatter', String, self.callback)
    
    def increase(self):
        self.count_msg = self.count_msg + 1
    
    def reset(self):
        self.count_msg = 0

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

        self.increase()
        if self.count_msg == 10:
            self.pub.publish(data)
            rospy.loginfo('count msg: ' + str(self.count_msg))
            self.reset()
            
    

def main():
    listener = Listener()


    rospy.init_node('listener', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()