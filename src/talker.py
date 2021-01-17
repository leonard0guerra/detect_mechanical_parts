#!/usr/bin/env python


## Simple talker that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

class Talker:

    def __init__(self):
        self.pub = rospy.Publisher('chatter', String, queue_size=10)

    def talk(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            rate.sleep()

def main():
    talker = Talker()
    rospy.init_node('talker', anonymous=True)

    try:
        talker.talk()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
