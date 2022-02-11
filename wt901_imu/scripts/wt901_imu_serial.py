#! /usr/bin/env python3

import serial
import struct
from enum import Enum
import rospy
from sensor_msgs.msg import Imu

class IMUData(Enum):
    ACCELERATION = 0x51
    ANGULAR_VEL = 0x52
    EULER = 0x53
    QUATERNION = 0x59

class WT901IMU:
    def __init__(self) -> None:
        self.constant = 32768.0
        self.acc_conv = self.constant * 16.0
        self.ang_vel_conv = self.constant * 2000.0
        self.euler_conv = self.constant * 180.0
        self.ser = serial.Serial('/dev/wt901IMU')
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "wt901_imu"
    
    def execute(self):
        data = self.ser.read(11)
        self.imu_msg.header.stamp = rospy.Time.now()
        if (data[1] == IMUData.ACCELERATION):
            x, y, z = [val / self.acc_conv for val in struct.unpack('<hhh', data[2:8])]
            self.imu_msg.linear_acceleration.x = y
            self.imu_msg.linear_acceleration.y = x
            self.imu_msg.linear_acceleration.z = -z
        elif (data[1] == IMUData.ANGULAR_VEL):
            x, y, z = [val / self.ang_vel_conv for val in struct.unpack('<hhh', data[2:8])]
            self.imu_msg.angular_velocity.x = y
            self.imu_msg.angular_velocity.y = x
            self.imu_msg.angular_velocity.z = -z
        elif (data[1] == IMUData.EULER):
            x, y, z = [val / self.ang_vel_conv for val in struct.unpack('<hhh', data[2:8])]
            print('x: %f y: %f z: %f' % (x, y, z))
        elif (data[1] == IMUData.QUATERNION):
            x, y, z = [val / self.ang_vel_conv for val in struct.unpack('<hhh', data[2:8])]
            self.imu_msg.angular_velocity.x = y
            self.imu_msg.angular_velocity.y = x
            self.imu_msg.angular_velocity.z = -z
        

if __name__ == '__main__':
    imu_device = WT901IMU()
    rospy.init_node('wt901_node', anonymous=False)
    pub = rospy.Publisher('imu/data_raw', Imu, latch=False, queue_size=5)
    msg = Imu()
    msg.header.frame_id = "wt901_imu"
    r = rospy.Rate(50) # 50hz

    while(not rospy.is_shutdown):
        imu_device.execute()
        pub.publish(imu_device.imu_msg)
        r.sleep()