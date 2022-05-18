#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from sonardyne_msgs.msg import SMS

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import sonardyne_usbl.modem


class SonardyneModemNode(object):
    def __init__(self):
        connection_type = rospy.get_param('~connection/type')
        if connection_type == 'serial':
            port = rospy.get_param('~connection/port')
            baud_rate = rospy.get_param('~connection/baud_rate', 9600)
            connection = sonardyne_usbl.modem.SerialConnection(port, baud_rate)
        elif connection_type == 'tcp':
            host = rospy.get_param('~connection/host')
            port = rospy.get_param('~connection/port', 4000)
            connection = sonardyne_usbl.modem.TCPConnection(host, port)
        elif connection_type == 'udp':
            host = rospy.get_param('~connection/host')
            input_port = rospy.get_param('~connection/input_port', 50011)
            output_port = rospy.get_param('~connection/outport_port', 50010)
            connection = sonardyne_usbl.modem.UDPConnection(input_port, output_port, host)

        self.modem = sonardyne_usbl.modem.Modem(connection)
        self.modem.enableDiagnostics()

        self.message_pub = rospy.Publisher('~received_sms', SMS, queue_size=10)
        self.raw_pub = rospy.Publisher('~raw', String, queue_size=10)

        send_sms_sub = rospy.Subscriber('~send_sms', SMS, self.sendSMSCallback, queue_size=10)
        send_raw_sub = rospy.Subscriber('~send_raw', String, self.sendRawCallback, queue_size=10)

        self.diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)

        timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def timerCallback(self, _event):
        data = self.modem.getResponse()
        if data is not None:
            print (data)
            self.raw_pub.publish(String(data['raw']))
            if 'response_type' in data and data['response_type'] == 'SMS' and 'message' in data:
                sms = SMS()
                sms.receive_time = rospy.Time.now()
                sms.address = data['address']
                sms.message = data['message']
                self.message_pub.publish(sms)
            if 'diag' in data:
                diag_array = DiagnosticArray()
                diag_array.header.stamp = rospy.Time.now()

                ds = DiagnosticStatus()
                ds.name = rospy.get_name()
                for k in data['diag'].keys():
                    ds.values.append(KeyValue(k,data['diag'][k]))
                diag_array.status.append(ds)
                self.diagnostic_pub.publish(diag_array)

    def sendSMSCallback(self, msg):
        self.modem.sendSMS(msg.address, msg.message)

    def sendRawCallback(self, msg):
        self.modem.send(msg.data+'\n')

if __name__ == '__main__':
    rospy.init_node('sonardyne_modem')
    modem_node = SonardyneModemNode()
    rospy.spin()

