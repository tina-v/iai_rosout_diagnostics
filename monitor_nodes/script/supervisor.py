#!/usr/bin/env python

import rospy
import logging

from drive_msgs.msg import SpeakingNumber
from drive_msgs.srv import GetStatistics, GetStatisticsRequest
from rosgraph_msgs.msg import Log
from rospy.service import logger
from drive_msgs.srv import GetLogger
from sets import Set

class SupervisorNode(object):
    def __init__(self):
        """
        initialisation of an object from SupervisorNode.
        subscribed to rosout (rosout_agg)
        """
        #msgsInfo = []
        self.sub = rospy.Subscriber("rosout_agg", Log, self.updateLoggerList)
        self.srv = rospy.Service("~get_logger_level", GetLogger, self.get_logger_level)
        self.loglist= []
        #self.loglist = np.array(msgsInfo)
        self.logger = logging.getLogger("rosout")


    def get_logger_level(self, srv):
        """
        returns List of existing Loggers (and their log_level)
        :param msg:
        :return: Nodes
        """
        for x in self.loglist:
            print(x)
        #return self.loglist
        #logger.setLevel(logging.INFO)
        #logger.setLevel(level)

    def updateLoggerList(self, msg):
        """
        Callback to ROS topic that processes new message.
        and saves the origin node of the msg in loglist.
        :param msg: The newest ROS message to process.
        :type msg: rosout
        :return: Nothing.
        :rtype: None
        """
        msgsInfo = [msg.name, msg.level]

        #for x in np.nditer(self.loglist):
        for x in self.loglist:
            if msgsInfo[0] == x[0]:
                if msgsInfo[1] > x[1]:
                    self.loglist[x, msgsInfo]
                    print("works!")
            elif x == self.loglist[-1]:
                self.loglist = np.concatenate((self.loglist, msgsInfo), axis = 0)

        # ______
        #
        #
        # for x in self.loglist:
        #     if msg.name in self.loglist[0::1]:
        #         present_msg_in_loglist = self.loglist[index_of_present_msg_in_loglist]
        #         if msg.level > present_msg_in_loglist[1]:
        #             self.loglist.insert(index_of_present_msg_in_loglist, msgsInfo)
        # else:
        #     self.loglist = np.append(self.loglist, msgsInfo)

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
    rospy.init_node('supervisor_node', anonymous=True)
    SupervisorNode()
    rospy.spin()
