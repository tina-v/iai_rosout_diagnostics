#!/usr/bin/env python

import rospy
from drive_msgs.msg import SpeakingNumber
from drive_msgs.srv import GetStatistics, GetStatisticsRequest


class StatisticsNode(object):
    def __init__(self):
        """
        initialisation of an object from StatisticsNode.
        """
        self.sub = rospy.Subscriber("~speaking_numbers", SpeakingNumber, self.updateCount)
        self.srv = rospy.Service("~get_statistics", GetStatistics, self.handleStatistics)
        self.first_msg = None
        self.last_msg = None
        self.msg_count = 0

    def updateCount(self, msg):
        """
        Callback to ROS topic that processes new SpeakingNumbers.
        :param msg: The newest ROS message to process.
        :type msg: SpeakingNumber
        :return: Nothing.
        :rtype: None
        """
        self.msg_count += 1

        if not self.first_msg:
            self.first_msg = msg

        self.last_msg = msg

    def handleStatistics(self, req):
        """
        service that returns the processes information of SpeakingNumber.
        :param req: New service request.
        :type req: GetCountRequest
        :return: number of messengers received, firstmsg and lastmsg received
        :rtype: dict
        """
        # return {'num_messages':self.msg_count,'first_message': self.first_msg, 'last_message': self.last_msg}


if __name__ == '__main__':
    rospy.init_node('getStatistic_node', anonymous=True)
    StatisticsNode()
    rospy.spin()
