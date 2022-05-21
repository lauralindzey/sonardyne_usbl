#!/usr/bin/env python3

import re
import rospy
from typing import Dict, List, Optional, Tuple

from std_msgs.msg import String
from sonardyne_msgs.msg import SMS

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from ros_acomms.msg import Packet, ReceivedPacket
from ros_acomms.srv import QueueTxPacket, QueueTxPacketRequest, QueueTxPacketResponse

import sonardyne_usbl.modem


class SonardyneModemNode(object):
    def __init__(self) -> None:
        # Sonardyne Telemetry Schemes; not all modems support all of them.
        self.valid_rates = [0,1,2,3,4,5,6,7,8]
        self.default_rate = int(rospy.get_param("~default_rate"))
        if self.default_rate not in self.valid_rates:
            msg = "{} is not a valid sonardyine Telemetry Scheme"
            raise Exception(msg)

        # In some network configurations, the packets coming from the
        # Sonardyne are split, such that a single message is split across
        # several socket.recv() calls. So, try to piece them back together.
        self.accumulated_packet = None

        try:
            self.sdyne_addr = rospy.get_param("~sdyne_addr")
        except KeyError:
            msg = "sdyne_addr is a required parameter"
            raise Exception(msg)
        except ValueError:
            msg = "sdyne_addr must be an int"
            raise Exception(msg)

        self.setupConnection()
        self.modem = sonardyne_usbl.modem.Modem(self.connection)

        self.modem.enableDiagnostics()

        self.message_pub = rospy.Publisher('~received_sms', SMS, queue_size=10)
        self.raw_pub = rospy.Publisher('~raw', String, queue_size=10)

        send_sms_sub = rospy.Subscriber('~send_sms', SMS, self.sendSMSCallback, queue_size=10)
        send_raw_sub = rospy.Subscriber('~send_raw', String, self.sendRawCallback, queue_size=10)

        self.diagnostic_pub = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size=10)

        # ros_acomms specific setup
        self.packet_rx_pub = rospy.Publisher('packet_rx', ReceivedPacket,
                                             queue_size=10)


        self.queue_tx_packet_service = rospy.Service(
            'queue_tx_packet', QueueTxPacket, self.handleQueueTxPacket)

        timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def setupConnection(self) -> None:
        connection_type = rospy.get_param('~connection/type')
        if connection_type == 'serial':
            port = rospy.get_param('~connection/port')
            baud_rate = rospy.get_param('~connection/baud_rate', 9600)
            self.connection = sonardyne_usbl.modem.SerialConnection(port, baud_rate)
        elif connection_type == 'tcp':
            host = rospy.get_param('~connection/host')
            port = rospy.get_param('~connection/port')
            self.connection = sonardyne_usbl.modem.TCPConnection(host, port)
        elif connection_type == 'udp':
            remote_host = rospy.get_param('~connection/remote_host')
            input_port = rospy.get_param('~connection/input_port')
            output_port = rospy.get_param('~connection/output_port')
            self.connection = sonardyne_usbl.modem.UDPConnection(input_port, output_port, remote_host)

    def timerCallback(self, _event: rospy.timer.TimerEvent) -> None:
        data = self.modem.getResponse()
        if data is not None:
            # The getResponse parsing is a bit complicated; the ros_acomms side of
            # things just needs raw byes, so we just do our own parsing.
            self.handleReceivedPacket(data['raw'])

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

    def sendSMSCallback(self, msg: SMS) -> None:
        self.modem.sendSMS(msg.address, msg.message)

    def sendRawCallback(self, msg: String) -> None:
        self.modem.send(msg.data+'\n')

    def parseSMS(self, msg: str) -> Tuple[Optional[int], Optional[bytes]]:
        # NOTE(lindzey): I think you can compile the regex ahead of time for
        #   some sort of performance improvement? (Not like this is high-rate...)

        # Hacky way of detecting whether the message is new-line terminated.
        # If it's not, then we probably have a case where it got split into two
        # reads from the socket.
        if msg == msg.strip():
            rospy.logerr("We probably have an incomplete message!")


        regex = r"^(?P<prelim>.*)SMS:(?P<addr>[0-9]{4})(?P<params>.*)\|(?P<data>[0-9A-F]*)"
        mm = re.match(regex, msg)
        if mm is None:
            print("Could not parse SMS data from reply: {}".format(msg))
            return None, None
        addr = int(mm['addr'])  # Should succeed, since regex matched to 4 digits
        data = mm['data']
        # If data is odd-length, then the hex ascii conversion will fail.
        # Go ahead and try to parse the bytes that did come in successfully.
        if len(data) % 2 == 1:
            data = data[:-1]
        try:
            # Given that the regex matches hex characters, this will only fail
            # if it's an odd-length string.
            data_bytes = bytes.fromhex(data)
        except ValueError:
            rospy.logerr("Could not convert {} chars to hex: {!r}"
                         .format(len(data), data))
            return addr, None
        print("Received {} bytes from {}".format(len(data_bytes), addr))


        packet = ReceivedPacket()
        packet.header.stamp = rospy.Time.now()
        packet.packet.src = addr
        packet.packet.dest = self.sdyne_addr  # TODO(lindzey): Add this to class?
        packet.packet.packet_type = Packet.PACKET_TYPE_UNKNOWN
        # Not sure how else to say "n/a"
        packet.packet.miniframe_rate = -1
        # The Sonardyne actually changes rates, but the received message
        # doesn't indicate what rate it was received on.
        packet.packet.dataframe_rate = -1
        # We don't have miniframes / dataframes, and ros_acomms assumes that
        # there will be data in the miniframes, so only using those.
        packet.packet.miniframe_bytes = data_bytes
        packet.packet.dataframe_bytes = []
        packet.minibytes_valid = [0, len(data_bytes)]
        packet.databytes_valid = []
        # The CST is micromodem specific, so don't fill that out.
        self.packet_rx_pub.publish(packet)

        return addr, data_bytes

    def isHexAscii(self, msg: str) -> bool:
        """
        Test whether input message contains only hex ascii characters.
        TODO(lindzey): When I have internet again, replace with actual library function
        """
        hex_chars = ['0','1','2','3','4','5','6','7','8','9',
                     'A', 'B', 'C', 'D', 'E', 'F']
        return all([ch in hex_chars for ch in msg])

    def handleReceivedPacket(self, raw_data: str) -> None:
        # Try to handle the case where a new packet is a continuation of
        # the previous one.
        if self.accumulated_packet is not None:
            rospy.loginfo("Last packet wasn't complete")
            if self.isHexAscii(raw_data.strip()):
                rospy.loginfo("...and can append new data to it!")
                self.accumulated_packet = self.accumulated_packet + raw_data
                # Was the new data terminated?
                if self.accumulated_packet != self.accumulated_packet.strip():
                    rospy.loginfo("Tryign to parse SMS")
                    self.parseSMS(self.accumulated_packet)
                    self.accumulated_packet = None
                # TODO(lindzey): This logic is ugly.
                return

            else:
                # Should try to send it anyways ... maybe the first bytes will be useful
                self.parseSMS(self.accumulated_packet)
                self.accumulated_packet = None

        # We don't care about non-SMS messages.
        if "SMS" not in raw_data:
            return


        # NOTE(lindzey): This might be where we add a state machine checking
        #    whether a given message was sent successfully.
        if 'FAILED' in raw_data:
            rospy.loginfo("SMS FAILED: {}".format(raw_data))
            return
        elif 'NO_REPLY' in raw_data:
            # Only applies if messages sent with A0 or A1
            rospy.loginfo("No reply to SMS. {}".format(raw_data))
            return
        elif 'ACK' in raw_data:
            rospy.loginfo("Received ACK. {}".format(raw_data))
            return
        elif 'OK' in raw_data:
            rospy.loginfo("Transmission confirmed: {}".format(raw_data))
            return

        if raw_data == raw_data.strip():
            # This is probably an incomplete packet!
            print("This is probably an incomplete packet -- not parsing yet")
            self.accumulated_packet = raw_data
        else:
            src_addr, data_bytes = self.parseSMS(raw_data)
            if data_bytes is None:
                rospy.logwarn("Could not parse SMS: {}".format(raw_data))
                return


    def handleQueueTxPacket(self, request: QueueTxPacketRequest) -> None:
        # NOTE(lindzey): This driver does not implement any sort of queue,
        #    and simply attempts to send a message as soon as a request is
        #    received. Unlike the Micromodem, we don't get a "transmission
        #    finished" message, so any sort of queueing would be based on
        #    a lookup table for rates + num_bytes -> transmission times.
        #  request.queue and request.insert_at_head are ignore

        # TODO(lindzey): Look into whether we can adjust the transmit power.
        # For now, simply ignore request.requested_src_level_db

        if request.packet.dataframe_rate not in self.valid_rates:
            rospy.logerr_once("Requested invalid rate {} for Sonardyne"
                              .format(request.packet.dataframe_rate))
            rate = self.default_rate
        else:
            rate = request.packet.dataframe_rate

        if len(request.packet.dataframe_bytes) > 0:
            rospy.logerr("Sonardyne only supports a single class of data, "
                         "but dataframe bytes were provided.")
            data = request.packet.miniframe_bytes + request.packet.dataframe_bytes
        else:
            data = request.packet.miniframe_bytes
        # Sonardyne requires uppercase in order to use 4-bit encoding of
        # the message chars.
        hex_data = data.hex().upper()

        # The Modem class's sendSMS isn't flexible enough, so roll our own.
        # Roland filters on messages and requires a <
        sms_cmd = "<SMS:{:04d};TS{},P0,C0,A3,R0,B0|{}\n".format(request.packet.dest, rate, hex_data)
        rospy.logerr("Sending command!: {}".format(sms_cmd))
        self.modem.send(sms_cmd)

        response = QueueTxPacketResponse()
        # We don't get immediate feedback about success in this thread.
        # So, for simplicity, always report true. The alternative is to
        # wait for a SMS:OK or SMS:FAILED, but I'm not sure how to store
        # this service call to match with data from another thread.
        response.success = True
        return response


if __name__ == '__main__':
    rospy.init_node('sonardyne_modem')
    modem_node = SonardyneModemNode()
    rospy.spin()

