# SPDX-License-Identifier: GPL-3.0-only

import time
import binascii
from threading import Lock

from revvy.utils.functions import retry


class TransportException(Exception):
    pass


def crc7(data, crc=0xFF):
    crc7_table = [
        0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
        0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
        0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
        0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
        0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
        0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
        0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
        0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
        0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
        0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
        0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
        0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
        0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
        0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
        0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
        0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
        0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
        0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
        0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
        0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
        0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
        0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
        0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
        0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
        0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
        0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
        0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
        0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
        0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
        0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
        0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
        0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79]

    for b in data:
        crc = crc7_table[(b ^ (crc << 1) & 0xFF)]
    return crc


class RevvyTransportInterface:
    def read(self, length): raise NotImplementedError()
    def write(self, data): raise NotImplementedError()


class Command:
    OpStart = 0
    OpRestart = 1
    OpGetResult = 2
    OpCancel = 3

    def __init__(self, op, command, payload=bytes()):
        self._op = op
        self._command = command
        self._payload = bytes(payload)

        if len(self._payload) > 255:
            raise ValueError('Payload is too long ({} bytes, 255 allowed)'.format(len(self._payload)))

    def get_bytes(self):
        header = bytes([self._op, self._command, len(self._payload)])
        payload_checksum = binascii.crc_hqx(self._payload, 0xFFFF)
        header += bytes(payload_checksum.to_bytes(2, byteorder='little'))
        header += bytes([crc7(header, 0xFF)])

        return header + self._payload

    @classmethod
    def start(cls, command, payload=bytes()):
        return cls(cls.OpStart, command, payload)

    @classmethod
    def get_result(cls, command):
        return cls(cls.OpGetResult, command)

    @classmethod
    def cancel(cls, command):
        return cls(cls.OpCancel, command)


class ResponseHeader:
    Status_Ok = 0
    Status_Busy = 1
    Status_Pending = 2

    Status_Error_UnknownOperation = 3
    Status_Error_InvalidOperation = 4
    Status_Error_CommandIntegrityError = 5
    Status_Error_PayloadIntegrityError = 6
    Status_Error_PayloadLengthError = 7
    Status_Error_UnknownCommand = 8
    Status_Error_CommandError = 9
    Status_Error_InternalError = 10

    StatusStrings = [
            "Ok",
            "Busy",
            "Pending",
            "Unknown operation",
            "Invalid operation",
            "Command integrity error",
            "Payload integrity error",
            "Payload length error",
            "Unknown command",
            "Command error",
            "Internal error"
        ]

    length = 5

    @staticmethod
    def is_valid_header(data):
        if len(data) >= ResponseHeader.length:
            checksum = crc7(data[0:ResponseHeader.length - 1])
            return checksum == data[4]

        return False

    def __init__(self, data):
        self._status = data[0]
        self._payload_length = data[1]
        self._payload_checksum = int.from_bytes(data[2:4], byteorder='little')
        self._header_checksum = data[4]

    def validate_payload(self, payload):
        return self._payload_checksum == binascii.crc_hqx(bytes(payload), 0xFFFF)

    def is_same_header(self, header):
        return len(header) >= self.length \
               and self._status == header[0] \
               and self._payload_length == header[1] \
               and self._payload_checksum == int.from_bytes(header[2:4], byteorder='little') \
               and self._header_checksum == header[4]

    @property
    def status(self):
        return self._status

    @property
    def payload_length(self):
        return self._payload_length


class Response:
    def __init__(self, status, payload):
        self._status = status
        self._payload = payload

    @property
    def status(self):
        return self._status

    @property
    def payload(self):
        return self._payload


class RevvyTransport:

    def __init__(self, transport: RevvyTransportInterface):
        self.timeout = 5  # [seconds] how long the slave is allowed to respond with "busy"
        self._transport = transport
        self._mutex = Lock()

    def send_command(self, command, payload=bytes()) -> Response:
        """Send a command and get the result."""
        with self._mutex:
            # once a command gets through and a valid response is read, this loop will exit
            while True:  # assume that integrity error is random and not caused by implementation differences
                # send command and read back status
                header = self._send_command(Command.start(command, payload))

                # wait for command execution to finish
                while header.status == ResponseHeader.Status_Pending:
                    header = self._send_command(Command.get_result(command))

                # check result
                # return a result even in case of an error, except when we know we have to resend
                if header.status != ResponseHeader.Status_Error_CommandIntegrityError:
                    response_payload = self._read_payload(header)
                    return Response(header.status, response_payload)

    def _read_response_header(self, retries=5):

        def _read_response_header_once():
            header_bytes = self._transport.read(ResponseHeader.length)
            has_valid_response = ResponseHeader.is_valid_header(header_bytes)
            if not has_valid_response:
                return False
            return ResponseHeader(header_bytes)

        header = retry(_read_response_header_once, retries)

        if not header:
            raise BrokenPipeError('Read response header: Retry limit reached')
        return header

    def _read_payload(self, header, retries=5):
        if header.payload_length == 0:
            return []

        def _read_payload_once():
            response_bytes = self._transport.read(header.length + header.payload_length)
            if ResponseHeader.is_valid_header(response_bytes):
                if not header.is_same_header(response_bytes):
                    raise ValueError('Read payload: Unexpected header received')

                payload_bytes = response_bytes[ResponseHeader.length:]
                has_valid_payload = header.validate_payload(payload_bytes)
                if has_valid_payload:
                    return payload_bytes

            return False

        payload = retry(_read_payload_once, retries)

        if not payload:
            raise BrokenPipeError('Read payload: Retry limit reached')

        return payload

    def _send_command(self, command: Command):
        """
        Send a command, wait for a proper response and return the response header
        """
        self._transport.write(command.get_bytes())
        start = time.time()
        while self.timeout == 0 or time.time() - start < self.timeout:
            response = self._read_response_header()
            if response.status != ResponseHeader.Status_Busy:
                return response
        raise TimeoutError
