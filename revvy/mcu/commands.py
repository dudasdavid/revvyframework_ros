# SPDX-License-Identifier: GPL-3.0-only

import struct
import traceback
from abc import ABC
from collections import namedtuple

from revvy.mcu.rrrc_transport import RevvyTransport, Response, ResponseHeader


class UnknownCommandError(Exception):
    pass


class Command:
    """A generic command towards the MCU"""
    def __init__(self, transport: RevvyTransport):
        self._transport = transport
        self._command_byte = self.command_id

    @property
    def command_id(self): raise NotImplementedError

    def _process(self, response: Response):
        if response.status == ResponseHeader.Status_Ok:
            return self.parse_response(response.payload)
        elif response.status == ResponseHeader.Status_Error_UnknownCommand:
            raise UnknownCommandError("Command not implemented: {}".format(self._command_byte))
        else:
            try:
                status = ResponseHeader.StatusStrings[response.status]
            except KeyError:
                status = 'Unknown status (code {})'.format(response.status)

            raise ValueError('Command status: "{}" with payload: {}'.format(status, repr(response.payload)))

    def _send(self, payload=None):
        """Send the command with the given payload and process the response"""
        if payload is None:
            payload = []
        response = self._transport.send_command(self._command_byte, payload)

        try:
            return self._process(response)
        except (UnknownCommandError, ValueError) as e:
            print('Error response for command: {0:X} with payload {1} (length {2})'
                  .format(self._command_byte, payload, len(payload)))
            raise e

    def __call__(self, *args):
        if args:
            raise NotImplementedError

        return self._send()

    def parse_response(self, payload):
        if payload:
            raise NotImplementedError

        return None


class PingCommand(Command):
    @property
    def command_id(self): return 0x00



class ReadHardwareVersionCommand(ReadVersionCommand):
    @property
    def command_id(self): return 0x01


class ReadFirmwareVersionCommand(ReadVersionCommand):
    @property
    def command_id(self): return 0x02


BatteryStatus = namedtuple('BatteryStatus', ['chargerStatus', 'main', 'motor'])


class SetMasterStatusCommand(Command):
    @property
    def command_id(self): return 0x04

    def __call__(self, status):
        # TODO: make this accept something meaningful
        return self._send([status])


class SetBluetoothStatusCommand(Command):
    @property
    def command_id(self): return 0x05

    def __call__(self, status):
        # TODO: make this accept something meaningful
        return self._send([status])


class ReadOperationModeCommand(Command):
    @property
    def command_id(self): return 0x06

    def parse_response(self, payload):
        # TODO: make this return something meaningful
        assert len(payload) == 1
        return int(payload[0])


class RebootToBootloaderCommand(Command):
    @property
    def command_id(self): return 0x0B


class ReadPortTypesCommand(Command, ABC):
    def parse_response(self, payload):
        return parse_string_list(payload)


class ReadMotorPortTypesCommand(ReadPortTypesCommand):
    @property
    def command_id(self): return 0x11


class ReadSensorPortTypesCommand(ReadPortTypesCommand):
    @property
    def command_id(self): return 0x21


class ReadRingLedScenarioTypesCommand(Command):
    @property
    def command_id(self): return 0x30

    def parse_response(self, payload):
        return parse_string_list(payload)


class ReadPortAmountCommand(Command, ABC):
    def parse_response(self, payload):
        assert len(payload) == 1
        return int(payload[0])


class ReadMotorPortAmountCommand(ReadPortAmountCommand):
    @property
    def command_id(self): return 0x10


class ReadSensorPortAmountCommand(ReadPortAmountCommand):
    @property
    def command_id(self): return 0x20


class SetPortTypeCommand(Command, ABC):
    def __call__(self, port, port_type_idx):
        return self._send([port, port_type_idx])


class SetMotorPortTypeCommand(SetPortTypeCommand):
    @property
    def command_id(self): return 0x12


class SetSensorPortTypeCommand(SetPortTypeCommand):
    @property
    def command_id(self): return 0x22


class SetRingLedScenarioCommand(Command):
    @property
    def command_id(self): return 0x31

    def __call__(self, scenario_idx):
        return self._send([scenario_idx])


class GetRingLedAmountCommand(Command):
    @property
    def command_id(self): return 0x32

    def parse_response(self, payload):
        assert len(payload) == 1
        return int(payload[0])


class SendRingLedUserFrameCommand(Command):
    @property
    def command_id(self): return 0x33

    def __call__(self, colors):
        rgb565_values = list(map(rgb_to_rgb565_bytes, colors))
        led_bytes = list(struct.pack("<" + "H" * len(rgb565_values), *rgb565_values))
        return self._send(led_bytes)


class ConfigureDrivetrain(Command):
    @property
    def command_id(self): return 0x1A

    def __call__(self, drivetrain_type, config):
        return self._send([drivetrain_type, *config])


class RequestDifferentialDriveTrainSpeedCommand(Command):
    @property
    def command_id(self): return 0x1B

    def __call__(self, left, right, power_limit=0):
        speed_cmd = list(struct.pack('<bffb', 1, left, right, power_limit))
        return self._send(speed_cmd)


class RequestDifferentialDriveTrainPositionCommand(Command):
    @property
    def command_id(self): return 0x1B

    def __call__(self, left, right, left_speed=0, right_speed=0, power_limit=0):
        pos_cmd = list(struct.pack('<bllffb', 0, left, right, left_speed, right_speed, power_limit))
        return self._send(pos_cmd)


class RequestDifferentialDriveTrainTurnCommand(Command):
    @property
    def command_id(self): return 0x1B

    def __call__(self, turn_angle, wheel_speed=0, power_limit=0):
        turn_cmd = list(struct.pack('<blfb', 3, turn_angle, wheel_speed, power_limit))
        return self._send(turn_cmd)


class SetPortConfigCommand(Command, ABC):
    def __call__(self, port_idx, config):
        return self._send([port_idx] + config)


class SetMotorPortConfigCommand(SetPortConfigCommand):
    @property
    def command_id(self): return 0x13


class SetSensorPortConfigCommand(SetPortConfigCommand):
    @property
    def command_id(self): return 0x23


class SetMotorPortControlCommand(Command):
    @property
    def command_id(self): return 0x14

    def __call__(self, port_idx, control):
        return self._send([port_idx] + control)


class ReadPortStatusCommand(Command, ABC):
    def __call__(self, port_idx):
        return self._send([port_idx])

    def parse_response(self, payload):
        """Return the raw response"""
        return payload


class ReadMotorPortStatusCommand(ReadPortStatusCommand):
    @property
    def command_id(self): return 0x15


class ReadSensorPortStatusCommand(ReadPortStatusCommand):
    @property
    def command_id(self): return 0x24


class McuStatusUpdater_ResetCommand(Command):
    @property
    def command_id(self): return 0x3A


class McuStatusUpdater_ControlCommand(Command):
    @property
    def command_id(self): return 0x3B

    def __call__(self, slot, is_enabled: bool):
        return self._send([slot, is_enabled])


class McuStatusUpdater_ReadCommand(Command):
    @property
    def command_id(self): return 0x3C

    def parse_response(self, payload):
        """Return the raw response"""
        return payload


class ErrorMemory_ReadCount(Command):
    @property
    def command_id(self): return 0x3D

    def parse_response(self, payload):
        assert len(payload) == 4
        return int.from_bytes(payload, byteorder='little')


class ErrorMemory_ReadErrors(Command):
    @property
    def command_id(self): return 0x3E

    def __call__(self, start_idx=0):
        return self._send(start_idx.to_bytes(4, byteorder='little'))

    def parse_response(self, payload):
        return payload


class ErrorMemory_Clear(Command):
    @property
    def command_id(self): return 0x3F


class ErrorMemory_TestError(Command):
    @property
    def command_id(self): return 0x40


# Bootloader-specific commands:
class InitializeUpdateCommand(Command):
    @property
    def command_id(self): return 0x08

    def __call__(self, crc, length):
        return self._send(list(struct.pack("<LL", crc, length)))


class SendFirmwareCommand(Command):
    @property
    def command_id(self): return 0x09

    def __call__(self, data):
        return self._send(data)


class FinalizeUpdateCommand(Command):
    @property
    def command_id(self): return 0x0A


def parse_string(data, ignore_errors=False):
    """
    >>> parse_string(b'foobar')
    'foobar'
    >>> parse_string([ord('f'), ord('o'), ord('o'), ord('b'), ord('a'), ord('r')])
    'foobar'
    >>> parse_string(b'foo\\xffbar', ignore_errors=True)
    'foobar'
    """
    return bytes(data).decode('utf-8', errors='ignore' if ignore_errors else 'strict')


def parse_string_list(data):
    """
    >>> parse_string_list(b'\x01\x06foobar')
    {'foobar': 1}
    """
    val = {}
    idx = 0
    while idx < len(data):
        key = data[idx]
        idx += 1
        sz = data[idx]
        idx += 1
        name = parse_string(data[idx:(idx + sz)])
        idx += sz
        val[name] = key
    return val


def rgb_to_rgb565_bytes(rgb):
    """
    Convert 24bit color to 16bit

    >>> rgb_to_rgb565_bytes(0)
    0
    >>> rgb_to_rgb565_bytes(0x800000)
    32768
    >>> rgb_to_rgb565_bytes(0x080408)
    2081
    >>> rgb_to_rgb565_bytes(0x808080)
    33808
    >>> rgb_to_rgb565_bytes(0xFFFFFF)
    65535
    """
    r = (rgb & 0x00F80000) >> 8
    g = (rgb & 0x0000FC00) >> 5
    b = (rgb & 0x000000F8) >> 3

    return r | g | b
