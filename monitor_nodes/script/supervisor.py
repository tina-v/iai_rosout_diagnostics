#!/usr/bin/env python

import rospy
import logging

from drive_msgs.msg import SpeakingNumber
from drive_msgs.srv import GetStatistics, GetStatisticsRequest
from rosgraph_msgs.msg import Log
from rospy.service import logger
from drive_msgs.srv import GetLogger
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

class SupervisorNode(object):
    def __init__(self):
        """
        initialisation of an object from SupervisorNode.
        subscribed to rosout (rosout_agg)
        publishes in diagnostic topic as diagnostic_msgs/DiagnosticArray
        when it receives a msg from rosout.
        """
        self.sub = rospy.Subscriber("rosout_agg", Log, self.update)
        self.srv = rospy.Service("~set_logger_level_of", GetLogger, self.set_logger_level_of_other_node)
        self.pub = rospy.Publisher("~diagnostic", DiagnosticArray,  queue_size=10)
        self.loglist = []
        self.logger = logging.getLogger("rosout")


    def set_logger_level_of_other_node(self,srv):
        """
        Setting a new level for a Node
        :param srv: GetLogger
        :return: None
        """
        logging.getLogger(srv.name).setLevel(srv.logger_level)


    def update(self, msg):
        """
        Callback to ROS topic that processes new message.
        and saves the origin node of the msg and the logger_level in loglist.
        :param msg: The newest ROS message to process.
        :type msg: rosout
        :return: Nothing
        :rtype: None
        """

        #mapping rosgraph.msg to DiganosticStatus.msg
        level = bytes
        name = 'supervisor_node'
        message = msg.msg
        hardware_id = msg.name
        values = [msg.file, msg.function]

        if msg.level <= 3:
            level = 0
        elif msg.level == 4:
            level = 1
        elif msg.level == 8:
            level = 2
        elif msg.level == 16:
            level = 3

        status = DiagnosticStatus(level, name, message, hardware_id, values)
        print(type(status))

        self.pub.publish(DiagnosticArray(msg.header, status))

        #TODO die abfrage ist das problem-
       # if not self.loglist:
        #    self.loglist.append(msgsInfo)
        #
        # for x in self.loglist:
        #     if x[0] == msgsInfo[0]:
        #         if msgsInfo[1] > x[1]:
        #             self.loglist.insert(self.loglist.index(x), msgsInfo)
        #     elif x == self.loglist[-1] or self.loglist[0]:
        #         self.loglist.append(msgsInfo)

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

if __name__ == '__main__':
    rospy.init_node('supervisor_node', anonymous=True)
    SupervisorNode()
    rospy.spin()
