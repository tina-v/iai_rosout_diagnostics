#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log


# printing number on console and log
class TestNode(object):
    def __init__(self):
        """
        publishes log msg through speaking_numberNode topic.
        """
        topic = '~TestNode'
        self.num = 0
        self.name_str = None
        self.rate = None
        self.rate = rospy.get_param('~speech_rate', 1)
        self.onlyInfo = None
        self.onlyInfo = rospy.get_param('~onlyInfo', False)

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

            if self.onlyInfo:
                rospy.loginfo("test_info" + self.name_str)
            else:
                rospy.loginfo("test_info" + self.name_str)
                rospy.logwarn("test_warning" + self.name_str)
                rospy.logfatal("test_fatal" + self.name_str)
                rospy.logerr("test_error" + self.name_str)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('test_node', anonymous=True)
    my_node = TestNode()
    rospy.sleep(1.0) # initial sleep time
    my_node.speaking()