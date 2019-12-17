#!/usr/bin/env python3

import rospy
import math

from sensor_msgs.msg import Imu
#from tf.transformations import quaternion_from_euler


degrees2rad = math.pi/180.0

rospy.init_node("revvy_imu_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

roll=0
pitch=0
yaw=0
seq=0



rospy.loginfo("Publishing IMU data...")

while not rospy.is_shutdown():
    yaw = 0*degrees2rad
    #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
    pitch = 0*degrees2rad
    roll = 0*degrees2rad

    # Publish message
    # AHRS firmware accelerations are negated
    # This means y and z are correct for ROS, but x needs reversing
    imuMsg.linear_acceleration.x = 0
    imuMsg.linear_acceleration.y = 0
    imuMsg.linear_acceleration.z = 0

    imuMsg.angular_velocity.x = 0
    #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
    imuMsg.angular_velocity.y = 0
    #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
    imuMsg.angular_velocity.z = 0

    #q = quaternion_from_euler(roll,pitch,yaw)
    q = [0,0,0,0]
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)





