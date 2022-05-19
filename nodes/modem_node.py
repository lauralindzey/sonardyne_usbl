#!/usr/bin/env python3

import binascii
import rospy

from std_msgs.msg import String
from sonardyne_msgs.msg import SMS

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from ros_acomms.msg import ReceivedPacket
from ros_acomms.srv import QueueTxPacket, QueueTxPacketResponse

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

        self.SRC = rospy.get_param('~SRC', 2501)
        self.DEST = rospy.get_param('~DEST', 2105)
        self.incoming_string = ""

        self.message_pub = rospy.Publisher('~received_sms', SMS, queue_size=10)
        self.raw_pub = rospy.Publisher('~raw', String, queue_size=10)

        send_sms_sub = rospy.Subscriber('~send_sms', SMS, self.sendSMSCallback, queue_size=10)
        send_raw_sub = rospy.Subscriber('~send_raw', String, self.sendRawCallback, queue_size=10)

        self.diagnostic_pub = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size=10)

        # ros_acomms specific setup
        self.packet_rx_pub = rospy.Publisher('~packet_rx', ReceivedPacket,
                                             queue_size=10)


        self.queue_tx_packet_service = rospy.Service(
            'queue_tx_packet', QueueTxPacket, self.handleQueueTxPacket)

        timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def timerCallback(self, event):
        data = self.modem.getResponse()
        if data is not None:
            print(data)
            self.modem_rx_handler(data)
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
                    ds.values.append(KeyValue(k, data['diag'][k]))
                diag_array.status.append(ds)
                self.diagnostic_pub.publish(diag_array)

    def sendSMSCallback(self, msg):
        self.modem.sendSMS(msg.address, msg.message)

    def sendRawCallback(self, msg):
        self.modem.send(msg.data+'\n')

    def pack_data(self, data):
        packet = data.hex().encode('ascii')
        return packet

    def scan_data(self, incoming_hex_string: str):
        rospy.logwarn("scan_data: Got {} new bytes".format(len(incoming_hex_string)))
        hex_chars = '0123456789abcdefABCDEF'
        new_hex_bytes = incoming_hex_string
        rospy.loginfo("new hex bytes: {}".format(new_hex_bytes))

        # Remove non-hex bytes and any remnants of str<->bytes conversion
        updated_hex_bytes = new_hex_bytes[1:int(len(new_hex_bytes))]
        rospy.logwarn("updated_hex_bytes: {}".format(updated_hex_bytes))
        clean_hex_bytes = "".join([c for c in updated_hex_bytes if c in hex_chars])

        self.incoming_string += clean_hex_bytes
        return self.incoming_string

    def modem_rx_handler(self, data):
        rxpacket = ReceivedPacket()
        rxpacket.header.stamp = rospy.Time.now()
        if data['response_type'] == 'SMS':
            if 'FAILED' in data['raw']:
                rospy.logerr("SMS FAILED, cannot parse")
                return
            else:
                print("data is {}".format(data['raw']))
                addrSplit = data['raw'].split('[')
                srcAddr = ''.join(x for x in addrSplit[0] if x.isdigit())
                rxpacket.packet.src = int(srcAddr)

                if 'NO_REPLY' in data['raw']:
                    rospy.logwarn("rx no reply from {}".format(rxpacket.packet.src))
                    return
                elif 'ACK' in data['raw']:
                    rospy.logwarn("RX ACK from {}".format(rxpacket.packet.src))
                    return
                else:
                    msgSplit = data['raw'].split('|') #split on | as per sdyne docs
                    rxMsg = self.scan_data(msgSplit[1]) #parse hex chars in SMS msg
                    rxMsgBytes = binascii.unhexlify(rxMsg) # unhex to get rx bytes
                    rxpacket.packet.miniframe_bytes = rxMsgBytes
                    rxpacket.cst.src = rxpacket.packet.src
                    self.packet_rx_pub.publish(rxpacket)


    def handleQueueTxPacket(self, request):
        rospy.loginfo(
            "handle_queue_tx_packet: Queueing new packet for transmission: {}".format(request))
        queue_tx_packet_resp = QueueTxPacketResponse()
        dest_address = str(request.packet.dest)
        packetToSend = self.pack_data(request.packet.miniframe_bytes)

        self.modem.sendSMS(dest_address, str(packetToSend))
        if request.insert_at_head:
            queue_tx_packet_resp.position_in_queue = 0
            queue_tx_packet_resp.success = True
        else:
            queue_tx_packet_resp.success = True
        return queue_tx_packet_resp


if __name__ == '__main__':
    rospy.init_node('sonardyne_modem')
    modem_node = SonardyneModemNode()
    rospy.spin()

