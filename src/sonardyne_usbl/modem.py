#!/usr/bin/env python3

import socket
import serial
import time
from typing import Any, Dict, Optional, Union

# TODO: mimic the ds_asio publication of all RawData to/from the port?

class TCPConnection(object):
    def __init__(self, host: str, port: int = 4000) -> None:
        self.host = host
        self.port = port
        self.open()

    def open(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.1)
        self.sock.connect((self.host, self.port))

    def send(self, data: str) -> None:
        self.sock.send(data.encode('utf-8'))

    # QUESTION(lindzey): Why convert away from bytes so early?
    #    sending binary data from SMS is just going to re-encode it.
    def recv(self) -> str:
        return self.sock.recv(10000).decode('utf-8')

class UDPConnection(object):
    def __init__(self,
                 inport: int,
                 outport: int,
                 remote_host: str,
                 local_address: str = '') -> None:
        self.inport = inport
        self.local_address = local_address
        self.remote_address = (remote_host, outport)
        self.open()

    def open(self) -> None:
        self.in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.in_socket.bind((self.local_address, self.inport))
        self.in_socket.settimeout(0.1)

        self.out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg: str) -> None:
        self.out_socket.sendto(msg.encode('utf-8'), self.remote_address)

    # QUESTION: Same issue as before ... why decode to a string?
    def recv(self) -> Optional[str]:
        try:
            return self.in_socket.recv(10000).decode('utf-8')
        except socket.timeout:
            pass
        return None

class SerialConnection(object):
    def __init__(self, device: str, baud:int = 9600) -> None:
        self.device = device
        self.baud = baud
        self.open()

    def open(self) -> None:
        self.port = serial.Serial(self.device, self.baud)
        self.port.timeout = 0.1

    def send(self, data: str) -> None:
        self.port.write(data.encode('utf-8'))

    def recv(self) -> Optional[str]:
        data = self.port.readline().decode('utf-8')
        if len(data):
            return data
        return None

class Modem(object):
    def __init__(self,
                 connection: Union[SerialConnection, UDPConnection, TCPConnection]
                ) -> None:
        self.connection = connection

    def send(self, data: str) -> None:
        print('sending:',data)
        self.connection.send(data)

    def recv(self) -> Optional[str]:
        return self.connection.recv()

    def enableDiagnostics(self) -> None:
        #self.send('DIAG:XC,DBV,SNR,FEC,TEL,DOP,RX1,IFL\n')
        self.send('DIAG:XC,DBV,SNR,IFL\n')

    def disableDiagnostics(self) -> None:
        self.send('DIAG:NONE\n')

    def hardwareSelfTest(self) -> None:
        self.send('CKHW\n')

    def getStatus(self, cmd: str, address: Optional[int] = None) -> None:
        send_cmd = cmd
        if address is not None:
            send_cmd += ':{:04d}'.format(address)
        self.send(send_cmd+'\n')

    # NOTE(lindzey): I'd love to see a better return data type =)
    #  I tried to annotate it for mypy, but have failed to make it error-free.
    #  * Most fields are strings, some of which can be None (response_type)
    #  * Some are bool ('external_power', 'tilt_over_45', 'override_flag')
    #  * Then, we have the battery dict of its own: List[Dict[str, Union[bool, str]]],
    #  * Finally, the return of ParseDiag: Dict[str,str]
    def getResponse(self):
        raw = self.recv()
        if raw is None:
            return None

        print('raw:',raw)

        ret = {}

        parts = raw.strip().split('|',1)
        data = parts[0]
        if len(parts) == 2:
            msg = parts[1]
            ret['message'] = msg

        diag = None
        if '[' in data:
            data,diag = data.strip().split('[',1)

        response_type = None
        UID = None

        for part in data.strip().split(','):
            if response_type is None:
                if part.startswith('>'):
                    if len(part) == 8 and part[1] == 'U':
                        UID = part[1:]
                    else:
                        response_type = part.split(':',1)[0][1:]
                else:
                    if len(part) == 7 and part[0] == 'U':
                        UID = part
                    else:
                        response_type = part.split(':',1)[0]

                if response_type == 'FS':
                    ret['response_type'] = 'fixed_status'
                elif response_type == 'CS':
                    ret['response_type'] = 'config_status'
                elif response_type == 'MS':
                    ret['response_type'] = 'modem_status'
                elif response_type == 'VS':
                    ret['response_type'] = 'volatile_status'
                else:
                    ret['response_type'] = response_type

                if response_type in ('FS','CS','MS','VS','SMS'):
                    ret['address'] = part.split(':',1)[1]

                if response_type == 'CKHW':
                    ret['test_result'] = part.split(':',1)[1]

            if response_type == 'FS':
                if len(part) == 7 and part[0] == 'U':
                    ret['unit_id'] = part[1:]
                if len(part) > 2 and part[:2] == 'FV':
                    ret['firmware_version'] = part[2:]
                if len(part) > 4 and part[:4] == 'TDR;':
                    ret['transducer_information'] = part[4:]

            if response_type == 'CS':
                if len(part) > 2 and part[:2] == 'LG':
                    ret['gain'] = part[2:]
                if len(part) > 3 and part[:3] == 'NPL':
                    ret['navigation_power_level'] = part[3:]
                if len(part) > 3 and part[:3] == 'SPL':
                    ret['start_power_level'] = part[3:]
                if len(part) > 3 and part[:3] == 'TPL':
                    ret['telemetry_power_level'] = part[3:]
                if len(part) > 3 and part[:3] == 'RXW':
                    ret['receive_wait_time'] = part[3:]

            if response_type == 'MS':
                if len(part) > 2 and part[:2] == 'MV':
                    ret['modem_version'] = part[2:]
                if len(part) > 2 and part[:2] == 'DD':
                    ret['data_delay'] = part[2:]
                if len(part) > 2 and part[:2] == 'MD':
                    ret['modem_delay'] = part[2:]
                if len(part) > 2 and part[:2] == 'UD':
                    ret['uplink_delay'] = part[2:]
                if len(part) > 2 and part[:2] == 'TS':
                    ret['telemetry_scheme'] = part[2:]
                if len(part) > 1 and part[:1] == 'P':
                    ret['port'] = part[1:]
                if len(part) > 2 and part[:2] == 'MR':
                    ret['master_retries'] = part[2:]
                if len(part) > 2 and part[:2] == 'SM':
                    ret['subframes_missed'] = part[2:]
                if len(part) > 3 and part[:3] == 'THR':
                    ret['threshold'] = part[3:]
                if len(part) > 3 and part[:3] == 'ICT':
                    ret['inter_character_time'] = part[3:]
                if len(part) > 2 and part[:2] == 'FQ':
                    ret['forward_queue'] = part[2:]
                if len(part) > 3 and part[:3] == 'MST':
                    ret['master_mode'] = part[3:]
                if len(part) > 2 and part[:2] == 'MU':
                    ret['multi_user'] = part[2:]
                if len(part) > 2 and part[:2] == 'FF':
                    ret['fire_and_forget'] = part[2:]
                if len(part) > 1 and part[:1] == 'B':
                    ret['buffer_size'] = part[1:]
                if len(part) > 1 and part[:1] == 'R':
                    ret['range'] = part[1:]
                if len(part) > 4 and part[:4] == 'DATA':
                    ret['data'] = part[4:]

            if response_type == 'VS':
                if len(part) > 3 and part[:3] == 'WKT':
                    ret['wake_up_tone'] = part[3:]
                if len(part) > 3 and part[:3] == 'HPR':
                    ret['hipap_channel'] = part[3:]
                if part == 'EXT':
                    ret['external_power'] = True
                if part == 'TILT':
                    ret['tilt_over_45'] = True
                if part == 'OV':
                    ret['override_flag'] = True
                if len(part) > 2 and part[:2] == 'BT':
                    battery: Dict[str, Union[bool, str]] = {}
                    bparts = part.split(';')
                    if len(bparts) > 1:
                        battery['type'] = bparts[1]
                    for bpart in bparts:
                        if len(bpart) > 2 and bpart[:2] == 'BT':
                            battery['number'] = bpart[2:]
                        if len(bpart) > 3 and bpart[:3] == 'VLT':
                            battery['voltage'] = bpart[3:]
                        if len(bpart) > 3 and bpart[:3] == 'IDC':
                            battery['current'] = bpart[3:]
                        if len(bpart) > 3 and bpart[:3] == 'CAP':
                            cap,remaining = bpart[3:].split('/',1)
                            battery['capacity'] = cap
                            battery['percent_remaining'] = remaining
                        if len(bpart) > 1 and bpart[:1] == 'T':
                            battery['temperature'] = bpart[1:]
                        if bpart == 'DIS':
                            battery['disconnected'] = True
                        if bpart == 'CHG':
                            battery['charging'] = True

                    if  'batteries' not in ret:
                        ret['batteries'] = []
                    # Mypy doesn't like this, because it can't tell that this entry
                    # is guaranteed to be a list.
                    ret['batteries'].append(battery)

        if diag is not None:
            ret['diag'] = self.parseDiag(diag[:-1])

        if UID is not None:
            ret['UID'] = UID

        ret['raw'] = raw

        # QUESTION(lindzey): Where does error detection and tracking come in?
        #    e.g. detecting a failed SMS? Right now, it's in Isaac's
        #    modifications, but I think that it would make more sense to put
        #    here.
        return ret

    def parseDiag(self, diag: str) -> Dict[str, str]:
        ret = {}
        parts = diag.split(',')
        for part in parts:
            if part.startswith('XC'):
                ret['cross_correlation'] = part[2:]
            if part.startswith('SNR'):
                ret['signal_to_noise_ratio'] = part[3:]
            if part.startswith('DBV'):
                ret['decibel_volts'] = part[3:]

        ret['raw'] = diag
        return ret

    def fixedStatus(self, address: Optional[int] = None) -> None:
        self.getStatus('FS', address)

    def configStatus(self, address:int = None) -> None:
        self.getStatus('CS', address)

    def volatileStatus(self, address: Optional[int] = None) -> None:
        self.getStatus('VS', address)

    def sendMessage(self, address: int, data: str) -> None:
        msg = "MDFT:{:04d}|{}\n".format(address, data)
        self.connection.send(msg)

    # NOTE(lindzey): This needs additional parameters:
    #    * ";TS#"  -- telemetry scheme
    #    * ",A#"  -- A3 for no reply, A0 for buffer, A1 for ack-only
    #    * ",R#"  -- only request ranges for data with reply (A0 or A1)
    #    * address really should be an int ...
    def sendSMS(self, address: str, data: str) -> None:
        msg = "SMS:{},R1|{}\n".format(address, data)
        self.connection.send(msg)

    def modemStatus(self) -> None:
        self.getStatus('MS')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("connection_type", type=str, help="Type of connection: serial, udp, tcp")
    parser.add_argument("--addr", type=int, help="This modem's address; used for status queries")
    parser.add_argument("--inport", type=int, help="UDP port to listen on")
    parser.add_argument("--outport", type=int, help="UDP port to send commands to")
    parser.add_argument("--remote_host", type=str, help="address to send commands to")
    parser.add_argument("--tcp_host", type=str, help="host for TCP connection")
    parser.add_argument("--device", type=str, help="Serial device (e.g. /dev/ttyUSB0)")
    parser.add_argument("--dest_addr", type=str, help="Address to send SMS to")
    parser.add_argument("--sms", type=str, help="Content of SMS message to send.")

    args = parser.parse_args()

    # Declaring this to make mypy happy
    connection: Union[SerialConnection, TCPConnection, UDPConnection]
    if args.connection_type == 'serial':
        connection = SerialConnection(args.device)
    elif args.connection_type == 'tcp':
        connection = TCPConnection(args.tcp_host)
    elif args.connection_type == 'udp':
        if args.inport is None:
            print("ERROR: a UDP connection requires specifying 'inport'")
        if args.outport is None:
            print("ERROR: a UDP connection requires specifying 'inport'")
        if args.remote_host is None:
            print("ERROR: a UDP connection requires specifying 'inport'")
        if args.inport and args.outport and args.remote_host:
            connection = UDPConnection(args.inport, args.outport, args.remote_host)
    else:
        print("ERROR: Must specify a connection type! (udp, tcp, serial)")

    modem = Modem(connection)
    modem.hardwareSelfTest()

    modem.enableDiagnostics()
    if args.addr is not None:
        modem.fixedStatus(args.address)
        modem.configStatus(args.address)
        modem.volatileStatus(args.address)
    modem.modemStatus()
    # NOTE(lindzey): This no longer works with UDP.
    if args.dest_addr is not None and args.sms is not None:
        modem.sendSMS(args.dest_addr, args.sms)

    while True:
        data = modem.getResponse()
        if data is not None and len(data):
            print (data)
        else:
            time.sleep(0)
