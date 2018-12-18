#!/usr/bin/env python

import rospy
# ROS message
# from speakingNumbersNodes import everything
from drive_msgs.msg import SpeakingNumber



# printing number on console and log
class SpeakingNumberNode(object):
    def __init__(self):
        """
        publishes SpeakingNumber msg through speaking_number topic.
        """
        topic = '~speaking_numbers'
        self.pub = rospy.Publisher(topic, SpeakingNumber,  queue_size=10)
        self.num = 0
        self.name_str = None
        self.rate = None
        #if rospy.has_param('speech_rate'):
        #    self.rate = rospy.get_param('~speech_rate', 1)
        #else:
        #    raise RuntimeError('Parameter ~speech_rate not found.')
        self.rate = rospy.get_param('~speech_rate', 1)

    def speaking(self):
        """
        starts to count in an 1 hz rate from 0 upwards and publishes it as SpeakingNumber.
        """
        rate = rospy.Rate(self.rate) # speaking rate
        while not rospy.is_shutdown():
            self.num += 1
            self.name_str = "count:" + str(self.num)
            rospy.logdebug(self.num)
            rospy.logdebug(self.name_str)

            rospy.loginfo("test_info" + self.name_str)
            rospy.logwarn("testwarning" + self.name_str)
            rospy.logfatal("test_fatal" + self.name_str)
            rospy.logerr("test_error" + self.name_str)
            self.pub.publish(SpeakingNumber(self.name_str, self.num))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('dummyDrive_node', anonymous=True)
    my_node = SpeakingNumberNode()
    rospy.sleep(1.0) # initial sleep time
    my_node.speaking()
