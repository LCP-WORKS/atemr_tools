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
    def __init__(self, publisher) -> None:
        self.constant = 32768.0
        self.acc_conv = self.constant * 16.0
        self.ang_vel_conv = self.constant * 2000.0
        self.euler_conv = self.constant * 180.0
        self.ser = serial.Serial('/dev/wt901IMU', 115200)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "wt901_imu"
        self.pub = publisher
    
    def execute(self):
        data = self.ser.read(11)
        self.imu_msg.header.stamp = rospy.Time.now()
        #print(IMUData.ACCELERATION.value)
        if (data[1] == IMUData.ACCELERATION.value):
            axes = struct.unpack("<hhh", data[2:8])
            x, y, z = [val / self.acc_conv for val in axes]
            self.imu_msg.linear_acceleration.x = y
            self.imu_msg.linear_acceleration.y = x
            self.imu_msg.linear_acceleration.z = -z
        elif (data[1] == IMUData.ANGULAR_VEL.value):
            axes = struct.unpack("<hhh", data[2:8])
            x, y, z = [val / self.ang_vel_conv for val in axes]
            self.imu_msg.angular_velocity.x = y
            self.imu_msg.angular_velocity.y = x
            self.imu_msg.angular_velocity.z = -z
        elif (data[1] == IMUData.EULER.value):
            axes = struct.unpack("<hhh", data[2:8])
            x, y, z = [val / self.ang_vel_conv for val in axes]
            print('x: %f y: %f z: %f' % (x, y, z))
        elif (data[1] == IMUData.QUATERNION.value):
            axes = struct.unpack("<hhh", data[2:8])
            x, y, z = [val / self.ang_vel_conv for val in axes]
            self.imu_msg.angular_velocity.x = y
            self.imu_msg.angular_velocity.y = x
            self.imu_msg.angular_velocity.z = -z
        
        self.pub.publish(self.imu_msg)
        

if __name__ == '__main__':
    rospy.init_node('wt901_node', anonymous=False)
    pub = rospy.Publisher('imu/data_raw', Imu, latch=False, queue_size=5)
    imu_device = WT901IMU(publisher=pub)
    r = rospy.Rate(50) # 50hz
    rospy.loginfo('Starting IMU node ...')
    while(not rospy.is_shutdown()):
        imu_device.execute()
        r.sleep()
