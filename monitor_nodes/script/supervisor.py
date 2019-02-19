#!/usr/bin/env python

import rospy
import logging

from rosgraph_msgs.msg import Log
from drive_msgs.srv import GetLogger
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.encoding import Config
from monitor_nodes.cfg import SupervisorDynamicReconfigConfig

import diagnostic_updater


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
        self.pub = rospy.Publisher("~diagnostics", DiagnosticArray,  queue_size=10)
        self.updater = diagnostic_updater.Updater()
        self.config_encoder = Config()
        self.dynamic_srv = Server(SupervisorDynamicReconfigConfig, self.callback)
        self.log_buffer = []
        self.list_of_allowed_node_logger_level = []
        self.logger = logging.getLogger("rosout")

    def set_pseudo_logger_level_of_other_node(self, node_name, new_level):
        """
        Setting restrictions by a level for a Node for entering the buffer and allowed to be published by supervisornode
        :param node_name: node of which level should be modified
        :param new_level: level in numeric
        :return: None
        """
        new_restriction = [node_name, new_level]
        for x in self.list_of_allowed_node_logger_level:
            if node_name == x[0] and new_level >= x[1]:
                self.list_of_allowed_node_logger_level.remove(x)
        self.list_of_allowed_node_logger_level.append(new_restriction)
        print(self.list_of_allowed_node_logger_level)

    def allowed_msg(self, nodename, level):
        """
        compares incomming_msg node with list of restrictions
        :return: boolean if the msg is allowed in the buffer
        """

        for x in self.list_of_allowed_node_logger_level:
            if nodename == x[0] and level < x[1]:
                return False
        return True

    def update_buffer(self, diagnostic_msg):
        """
        adding new msg to buffer.msg which stays in buffer for 3 to 5 secs before it gets removed.
        :param diagnostic_msg: new msg to be added to the buffer of the supervisor_node (msg of type DiagnosticArray)
        :return: void
        """
        self.log_buffer.append(diagnostic_msg)
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

            # mapping rosgraph.msg to DiganosticStatus.msg
            level = bytes
            name = msg.name
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

            if self.allowed_msg(hardware_id, level):
                self.update_buffer(new_msg)
                self.pub.publish(new_msg)
                # updater.update() -is getting bugfixes from diagnosticstatus
                self.updater.publish(status_array)
            self.updateConfig(name, level)

    def updateConfig(self, node_name, level):
        # update_group = {"node_and_level": {"node": name}}
        # self.dynamic_srv.update_configuration(update_group)
        # as test- only node1!!!!!
        if node_name == "/speaking_numbers_node1" or self.config_encoder.get("Node") == "":
            if self.config_encoder.get("Level") > level:
                self.dynamic_srv.update_configuration({"Node": node_name, "Level": level})
                print(self.dynamic_srv.description)

    def callback(self, config, level):
        print(config.items())
        #not nice ...perhaps only items - instead of rewriting the whole config
        self.config_encoder = config
        if (config.get("Node") != ""):
            node_name = config.get("Node")
            new_level = config.get("Level")
            self.set_pseudo_logger_level_of_other_node(node_name, new_level)
        return config


if __name__ == '__main__':
    rospy.init_node('supervisor_node', anonymous=True)
    SupervisorNode()
    rospy.spin()
