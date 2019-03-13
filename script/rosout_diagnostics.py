#!/usr/bin/env python

import rospy

from rosgraph_msgs.msg import Log
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


class SupervisorNode(object):

    def __init__(self):
        """
        initialisation of an object from SupervisorNode.
        subscribed to rosout (rosout_agg)
        publishes in diagnostic topic as diagnostic_msgs/DiagnosticArray
        when it receives a msg from rosout.
        """

        self.sub = rospy.Subscriber("rosout_agg", Log, self.update)
        self.pub = rospy.Publisher("diagnostics", DiagnosticArray,  queue_size=10)
        self.level_mapping = {
            Log.INFO: DiagnosticStatus.OK,
            Log.DEBUG: DiagnosticStatus.OK,
            Log.WARN: DiagnosticStatus.WARN,
            Log.ERROR: DiagnosticStatus.ERROR,
            Log.FATAL: DiagnosticStatus.ERROR}

    def to_diagnostics_msg(self, msg):
        """

        :param msg:
        :return:
        :rtype: DiagnosticArray
        """
        level = self.level_mapping[msg.level]
        name = msg.name
        message = msg.msg
        hardware_id = msg.name
        values = [KeyValue('file', msg.file),
                  KeyValue('function', msg.function),
                  KeyValue('line', '{}'.format(msg.line))]

        status_array = [DiagnosticStatus(level, name, message, hardware_id, values)]
        msg.header.frame_id = rospy.get_name()
        return DiagnosticArray(msg.header, status_array)

    def update(self, msg):
        """
        Callback to ROS topic that processes new message.
        and saves the origin node of the msg and the logger_level in loglist.
        :param msg: The newest ROS message to process.
        :type msg: rosout
        :return: Nothing
        :rtype: None
        """
        if msg.name != rospy.get_name():
            self.pub.publish(self.to_diagnostics_msg(msg))


if __name__ == '__main__':
    rospy.init_node('rosout_diagnostics', anonymous=True)
    SupervisorNode()
    rospy.spin()
