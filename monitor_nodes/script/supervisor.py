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
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Header

class SupervisorNode(object):


    def __init__(self):
        """
        initialisation of an object from SupervisorNode.
        subscribed to rosout (rosout_agg)
        publishes in diagnostic topic as diagnostic_msgs/DiagnosticArray
        when it receives a msg from rosout.
        """

        self.sub = rospy.Subscriber("rosout_agg", Log, self.update)
        self.srv = rospy.Service("~set_logger_level_of", GetLogger, self.set_pseudo_logger_level_of_other_node)
        self.pub = rospy.Publisher("~diagnostic", DiagnosticArray,  queue_size=10)
        self.log_buffer = []
        self.list_of_allowed_node_logger_level = []
        self.logger = logging.getLogger("rosout")

#working on set_logger_level right now - noted problem: list duplicated nodenames
    #reflexion: getLogger ist nicht der node logger wie zuvor gedacht.

    #idea! total revamp - let's use that buffer- which means: saving node name and level in a list again....
    def set_pseudo_logger_level_of_other_node(self, node_name, new_level):
        """
        Setting restrictions by a level for a Node for entering the buffer and allowed to be published by supervisor node.
        :param node_name: node of which level should be modified
        :param new_level: level in numeric
        :return: None
        """
        new_restriction = [node_name, new_level]
        for x in self.list_of_allowed_node_logger_level:
            if node_name == x[0]:
                self.list_of_allowed_node_logger_level.remove(x)
                self.list_of_allowed_node_logger_level.append(new_restriction)
        self.list_of_allowed_node_logger_level.append(new_restriction)
        print(self.list_of_allowed_node_logger_level)
        # logging.getLogger(node_name).setLevel(new_level)


    """
    compares incomming_msg node with list of restrictions
    :return: boolean if the msg is allowed in the buffer
    """
    def allowed_msg(self, nodename, level):
        for x in self.list_of_allowed_node_logger_level:
            if nodename == x[0]:
                if level >= x[1]:
                    return True
            return False
        return True


    def update_buffer(self, diagnostic_msg):
        """
        adding new msg to buffer.msg which stays in buffer for 3 to 5 secs before it gets removed.
        :param diagnostic_msg: new msg to be added to the buffer of the supervisor_node (msg of type DiagnosticArray)
        :return: void
        """
        self.log_buffer.append(diagnostic_msg)
        #stamp.nsec: nanoseconds since stamp_secs OR stamp.sec: seconds (stamp_secs) since epoch?
        #prefering stamp.sec
        last_msg_in_buffer = self.log_buffer[0]
        if self.log_buffer[0]:
            time_of_last_msg_in_buffer = diagnostic_msg.header.stamp.secs - last_msg_in_buffer.header.stamp.secs
        # for now it's 3-5 secs buffer
        if time_of_last_msg_in_buffer > 3:
            for msg in self.log_buffer:
                if msg.header.stamp <= last_msg_in_buffer.header.stamp:
                    self.log_buffer.remove(msg)


    def update(self, msg):
        """
        Callback to ROS topic that processes new message.
        and saves the origin node of the msg and the logger_level in loglist.
        :param msg: The newest ROS message to process.
        :type msg: rosout
        :return: Nothing
        :rtype: None
        """
        this_node = rospy.get_name()
        if msg.name != this_node:

        #mapping rosgraph.msg to DiganosticStatus.msg
            level = bytes
            name = 'supervisor_node'
            message = msg.msg
            hardware_id = msg.name
            values = [KeyValue(msg.file, msg.function)]

            if msg.level <= 3:
                level = 0
            elif msg.level == 4:
                level = 1
            elif msg.level == 8:
                level = 2
            elif msg.level == 16:
                level = 3

            status_array = [DiagnosticStatus(level, name, message, hardware_id, values)]
            msg.header.frame_id = "supervisor"
            new_msg = DiagnosticArray(msg.header, status_array)

            #testing
            self.set_pseudo_logger_level_of_other_node(hardware_id, 16)

            if self.allowed_msg(hardware_id, level):
                self.update_buffer(new_msg)
                self.pub.publish(new_msg)




if __name__ == '__main__':
    rospy.init_node('supervisor_node', anonymous=True)
    SupervisorNode()
    rospy.spin()
