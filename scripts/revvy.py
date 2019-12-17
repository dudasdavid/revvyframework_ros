#!/usr/bin/env python3
from revvy.hardware_dependent.rrrc_transport_i2c import RevvyTransportI2C
from revvy.mcu.rrrc_control import *
from revvy.utils.thread_wrapper import periodic

import time
import math

import rospy
from std_msgs.msg import String, Int16, Int32, Int16MultiArray
from sensor_msgs.msg import Imu


Motors = {
    'NotConfigured': {'driver': 'NotConfigured', 'config': {}},
    'RevvyMotor':    {
        'driver': 'DcMotor',
        'config': {
            'speed_controller':    [1 / 35, 0.25, 0, -100, 100],
            'position_controller': [4, 0, 0, -600, 600],
            'acceleration_limits': [14400, 3600],
            'encoder_resolution':  1536
        }
    },
    'RevvyMotor_CCW': {
        'driver': 'DcMotor',
        'config': {
            'speed_controller':    [1 / 35, 0.25, 0, -100, 100],
            'position_controller': [4, 0, 0, -600, 600],
            'acceleration_limits': [14400, 3600],
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

(posP, posI, posD, speedLowerLimit, speedUpperLimit) = port_config['position_controller']
(speedP, speedI, speedD, powerLowerLimit, powerUpperLimit) = port_config['speed_controller']
(decMax, accMax) = port_config['acceleration_limits']

config = []
config += list(struct.pack("<h", port_config['encoder_resolution']))
config += list(struct.pack("<{}".format("f" * 5), posP, posI, posD, speedLowerLimit, speedUpperLimit))
config += list(struct.pack("<{}".format("f" * 5), speedP, speedI, speedD, powerLowerLimit, powerUpperLimit))
config += list(struct.pack("<ff", decMax, accMax))

drivetrainMotors = [1, 1, 1, 2, 2, 2]  # set all to drivetrain LEFT = 1, RIGHT = 2

leftSpeed, rightSpeed, lastLeftSpeed, lastRightSpeed = 0, 0, 0, 0
lastReadTime = 0


motorPortData = {"raw":[],"pos":0, "speed":0, "power":0, "pos_reached":0}
sensorPortData = {"raw":[]}
batteryData = {"raw":[],"brain":0,"motor":0}
accelerometerData = {"raw":[],"x":0,"y":0,"z":0}
gyroData = {"raw":[],"x":0,"y":0,"z":0}
yawData = {"raw":[]}
restartData = {"raw":[]}

sensorData = [motorPortData.copy(), motorPortData.copy(), motorPortData.copy(), motorPortData.copy(), motorPortData.copy(), motorPortData.copy(), sensorPortData.copy(), sensorPortData.copy(), sensorPortData.copy(), sensorPortData.copy(), batteryData, accelerometerData, gyroData, yawData, restartData]
degrees2rad = math.pi/180.0

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

def processIMUData(slot,lsb_value):
    raw = sensorData[slot]["raw"]
    (x, y, z) = struct.unpack('<hhh', bytes(raw))
    sensorData[slot]["x"] = x * lsb_value
    sensorData[slot]["y"] = y * lsb_value
    sensorData[slot]["z"] = z * lsb_value


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
            elif slot == 10:
                sensorData[slot]["brain"] = sensorData[slot]["raw"][1]
                sensorData[slot]["motor"] = sensorData[slot]["raw"][3]
            elif slot == 11:
                processIMUData(slot, 0.061*0.00981)
            elif slot == 12:
                processIMUData(slot, 0.035*degrees2rad)

        else:
            print('McuStatusUpdater: invalid slot length')

        idx = data_end

def robotCommThread():
    global leftSpeed, rightSpeed,lastLeftSpeed,lastRightSpeed

    if leftSpeed != lastLeftSpeed or rightSpeed != lastRightSpeed:
        robot_control.set_drivetrain_speed(leftSpeed, rightSpeed)
        lastLeftSpeed = leftSpeed
        lastRightSpeed = rightSpeed
    else:
        #robot_control.ping()
        data = robot_control.status_updater_read()
        processSensorData(data)
        #print("L:%d,R:%d" % (sensorData[0]["pos"],sensorData[3]["pos"]))


def publisherThread():
    global pubLeft, pubRight, pubImu
    global seq

    pubLeft.publish(Int32(-1*sensorData[0]["pos"]))
    pubRight.publish(Int32(sensorData[3]["pos"]))

    imuMsg.linear_acceleration.x = accelerometerData["x"]
    imuMsg.linear_acceleration.y = accelerometerData["y"]
    imuMsg.linear_acceleration.z = accelerometerData["z"]

    imuMsg.angular_velocity.x = gyroData["x"]
    imuMsg.angular_velocity.y = gyroData["y"]
    imuMsg.angular_velocity.z = gyroData["z"]

    q = [0,0,0,0]
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pubImu.publish(imuMsg)

    print(yawData["raw"])


def setSpeeds(data):
    global leftSpeed, rightSpeed

    try:
        leftSpeed = -data.data[0]
        rightSpeed = data.data[1]

    except rospy.ROSInterruptException:
        pass



pubLeft = rospy.Publisher('lwheel_ticks', Int32, queue_size=1)
pubRight = rospy.Publisher('rwheel_ticks', Int32, queue_size=1)
pubImu = rospy.Publisher('imu', Imu, queue_size=1)

rospy.init_node('revvyframework', anonymous=True)
rospy.Subscriber('wheels_desired_rate', Int16MultiArray, setSpeeds)

imuMsg = Imu()

imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

roll=0
pitch=0
yaw=0
seq=0


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

    robot_control.status_updater_control(0, True) # left motor
    robot_control.status_updater_control(3, True) # right motor
    robot_control.status_updater_control(10, True) # battery
    robot_control.status_updater_control(11, True) # acc
    robot_control.status_updater_control(12, True) # gyro
    robot_control.status_updater_control(13, True) # yaw data

    i2cThread = periodic(robotCommThread, 0.05, "Comm")  # 50ms
    i2cThread.start()

    pubThread = periodic(publisherThread, 0.05, "Pub")
    pubThread.start()

    input("Press any key to exit!")

    '''
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("!!! Keyboard exit !!!")
        i2cThread.exit()
        pubThread.exit()
    '''

