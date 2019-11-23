from revvy.hardware_dependent.rrrc_transport_i2c import RevvyTransportI2C
from revvy.mcu.commands import *
import time

class RevvyControl:
    def __init__(self, transport: RevvyTransport):
        self.ping = PingCommand(transport)

        self.set_master_status = SetMasterStatusCommand(transport)

        self.get_hardware_version = ReadHardwareVersionCommand(transport)
        self.get_firmware_version = ReadFirmwareVersionCommand(transport)

        self.get_motor_port_amount = ReadMotorPortAmountCommand(transport)
        self.get_motor_port_types = ReadMotorPortTypesCommand(transport)
        self.set_motor_port_type = SetMotorPortTypeCommand(transport)
        self.set_motor_port_config = SetMotorPortConfigCommand(transport)
        self.set_motor_port_control_value = SetMotorPortControlCommand(transport)
        self.get_motor_position = ReadMotorPortStatusCommand(transport)

        self.configure_drivetrain = ConfigureDrivetrain(transport)
        self.set_drivetrain_position = RequestDifferentialDriveTrainPositionCommand(transport)
        self.set_drivetrain_speed = RequestDifferentialDriveTrainSpeedCommand(transport)
        self.drivetrain_turn = RequestDifferentialDriveTrainTurnCommand(transport)

        self.get_sensor_port_amount = ReadSensorPortAmountCommand(transport)
        self.get_sensor_port_types = ReadSensorPortTypesCommand(transport)
        self.set_sensor_port_type = SetSensorPortTypeCommand(transport)
        self.set_sensor_port_config = SetSensorPortConfigCommand(transport)
        self.get_sensor_port_value = ReadSensorPortStatusCommand(transport)


Motors = {
    'NotConfigured': {'driver': 'NotConfigured', 'config': {}},
    'RevvyMotor':    {
        'driver': 'DcMotor',
        'config': {
            'speed_controller':    [1 / 37.5, 0.3, 0, -100, 100],
            'position_controller': [10, 0, 0, -900, 900],
            'position_limits':     [0, 0],
            'encoder_resolution':  1536
        }
    },
    'RevvyMotor_CCW': {
        'driver': 'DcMotor',
        'config': {
            'speed_controller':    [1 / 37.5, 0.3, 0, -100, 100],
            'position_controller': [10, 0, 0, -900, 900],
            'position_limits':     [0, 0],
            'encoder_resolution': -1536
        }
    }
}

Sensors = {
    'NotConfigured': {'driver': 'NotConfigured', 'config': {}},
    'HC_SR04':       {'driver': 'HC_SR04', 'config': {}},
    'BumperSwitch':  {'driver': 'BumperSwitch', 'config': {}},
}

port_config = Motors["RevvyMotor"]["config"]

(posMin, posMax) = port_config['position_limits']
(posP, posI, posD, speedLowerLimit, speedUpperLimit) = port_config['position_controller']
(speedP, speedI, speedD, powerLowerLimit, powerUpperLimit) = port_config['speed_controller']

config = list(struct.pack("<ll", posMin, posMax))
config += list(struct.pack("<{}".format("f" * 5), posP, posI, posD, speedLowerLimit, speedUpperLimit))
config += list(struct.pack("<{}".format("f" * 5), speedP, speedI, speedD, powerLowerLimit, powerUpperLimit))
config += list(struct.pack("<h", port_config['encoder_resolution']))

print(config)

with RevvyTransportI2C() as transport:
    robot_control = RevvyControl(transport.bind(0x2D))

    print(robot_control.get_firmware_version())
    print(robot_control.get_motor_port_amount())
    print(robot_control.get_sensor_port_amount())

    robot_control.set_master_status(3)


    robot_control.set_motor_port_type(4,1)
    robot_control.set_motor_port_config(4, config)

    while True:
        robot_control.ping()
        time.sleep(0.02)
