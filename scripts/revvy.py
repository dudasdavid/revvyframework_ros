#!/usr/bin/env python3
from revvy.hardware_dependent.rrrc_transport_i2c import RevvyTransportI2C
from revvy.mcu.rrrc_control import *
from revvy.thread_wrapper import periodic

import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

Motors = {
    'NotConfigured': {'driver': 'NotConfigured', 'config': {}},
    'RevvyMotor': {
        'driver': 'DcMotor',
        'config': {
            'speed_controller': [1 / 37.5, 0.3, 0, -100, 100],
            'position_controller': [10, 0, 0, -900, 900],
            'position_limits': [0, 0],
            'encoder_resolution': 1536
        }
    },
    'RevvyMotor_CCW': {
        'driver': 'DcMotor',
        'config': {
            'speed_controller': [1 / 37.5, 0.3, 0, -100, 100],
            'position_controller': [10, 0, 0, -900, 900],
            'position_limits': [0, 0],
            'encoder_resolution': -1536
        }
    }
}

Sensors = {
    'NotConfigured': {'driver': 'NotConfigured', 'config': {}},
    'HC_SR04': {'driver': 'HC_SR04', 'config': {}},
    'BumperSwitch': {'driver': 'BumperSwitch', 'config': {}},
}



port_config = Motors["RevvyMotor"]["config"]

(posMin, posMax) = port_config['position_limits']
(posP, posI, posD, speedLowerLimit, speedUpperLimit) = port_config['position_controller']
(speedP, speedI, speedD, powerLowerLimit, powerUpperLimit) = port_config['speed_controller']

config = list(struct.pack("<ll", posMin, posMax))
config += list(struct.pack("<{}".format("f" * 5), posP, posI, posD, speedLowerLimit, speedUpperLimit))
config += list(struct.pack("<{}".format("f" * 5), speedP, speedI, speedD, powerLowerLimit, powerUpperLimit))
config += list(struct.pack("<h", port_config['encoder_resolution']))

drivetrainMotors = [1, 1, 1, 2, 2, 2]  # set all to drivetrain LEFT = 1, RIGHT = 2

leftSpeed, rightSpeed,lastLeftSpeed,lastRightSpeed = 0, 0, 0, 0
lastReadTime = 0


motorPortData = {"raw":[],"pos":0, "speed":0, "power":0, "pos_reached":0}
sensorPortData = {"raw":[]}
batteryData = {"raw":[],"brain":0,"motor":0}
accelerometerData = {"raw":[]}
gyroData = {"raw":[]}
yawData = {"raw":[]}

sensorData = [motorPortData, motorPortData, motorPortData, motorPortData, motorPortData, motorPortData, sensorPortData, sensorPortData, sensorPortData, sensorPortData, batteryData, accelerometerData, gyroData, yawData]


def processMotorData(slot):
    raw = sensorData[slot]["raw"]
    if len(raw) == 9:
        (power, pos, speed) = struct.unpack('<blf', bytearray(raw))
        pos_reached = None
    elif len(raw) == 10:
        (power, pos, speed, pos_reached) = struct.unpack('<blfb', bytearray(raw))
    else:
        print('Slot {}: Received {} bytes of data instead of 9 or 10'.format(slot, len(raw)))
        return

    sensorData[slot]["pos"] = pos
    sensorData[slot]["speed"] = speed
    sensorData[slot]["power"] = power
    sensorData[slot]["pos_reached"] = pos_reached

def processSensorData(data):

    idx = 0
    while idx < len(data):
        slot = data[idx]
        slot_length = data[idx + 1]

        data_start = idx + 2
        data_end = idx + 2 + slot_length

        if data_end <= len(data):
            sensorData[slot]["raw"] = (data[data_start:data_end])
            if slot < 6:
                processMotorData(slot)
        else:
            print('McuStatusUpdater: invalid slot length')

        idx = data_end

def robotCommThread():
    global leftSpeed, rightSpeed,lastLeftSpeed,lastRightSpeed,lastReadTime

    if leftSpeed != lastLeftSpeed or rightSpeed != lastRightSpeed:
        robot_control.set_drivetrain_speed(leftSpeed, rightSpeed)
        lastLeftSpeed = leftSpeed
        lastRightSpeed = rightSpeed
    else:
        robot_control.ping()

    if time.time() - lastReadTime > 1:
        lastReadTime = time.time()
        data = robot_control.status_updater_read()
        processSensorData(data)
        print(sensorData[0]["pos"])




def controlCallback(data):
    global leftSpeed, rightSpeed
    #print(data.linear.x)
    #print(data.angular.z)
    #print(10 * "*")
    try:
        if data.angular.z != 0:
            leftSpeed = data.angular.z
            rightSpeed = data.angular.z
        else:
            leftSpeed = -data.linear.x
            rightSpeed = data.linear.x
    except rospy.ROSInterruptException:
        pass


rospy.init_node('revvyframework', anonymous=True)
rospy.Subscriber('key_vel', Twist, controlCallback)

with RevvyTransportI2C() as transport:
    robot_control = RevvyControl(transport.bind(0x2D))

    print("MCU Firmware version: %s" % robot_control.get_firmware_version())
    print("Number of motor ports: %s" % robot_control.get_motor_port_amount())
    print("Number of sensor ports: %s" % robot_control.get_sensor_port_amount())

    robot_control.set_master_status(3) # Set master LED green and monitoring communication

    robot_control.set_motor_port_type(1, 1)  # 0 ='NotConfigured': NullMotor, 1 = 'DcMotor': DcMotorController
    robot_control.set_motor_port_config(1, config)

    robot_control.set_motor_port_type(2, 1)  # 0 ='NotConfigured': NullMotor, 1 = 'DcMotor': DcMotorController
    robot_control.set_motor_port_config(2, config)

    robot_control.set_motor_port_type(4, 1)  # 0 ='NotConfigured': NullMotor, 1 = 'DcMotor': DcMotorController
    robot_control.set_motor_port_config(4, config)

    robot_control.set_motor_port_type(5, 1)  # 0 ='NotConfigured': NullMotor, 1 = 'DcMotor': DcMotorController
    robot_control.set_motor_port_config(5, config)

    robot_control.configure_drivetrain(1, drivetrainMotors)  # DIFFERENTIAL = 1

    robot_control.status_updater_control(0, True)
    robot_control.status_updater_control(3, True)
    robot_control.status_updater_control(10, True)
    robot_control.status_updater_control(11, True)
    robot_control.status_updater_control(12, True)
    robot_control.status_updater_control(13, True)

    thread = periodic(robotCommThread, 0.05, "Comm")  # 20ms
    thread.start()

    rospy.spin()
