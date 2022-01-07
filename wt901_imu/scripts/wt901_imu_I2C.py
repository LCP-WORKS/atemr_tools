#!/usr/bin/env python3
import smbus
import time
import rospy
from sensor_msgs.msg import Imu

do_config = False
debug = True
 
class Gyro(object):
 
    def __init__(self, addr):
        self.addr = addr
        self.i2c = smbus.SMBus(1)
        self.begin_config = list(0x88, 0xB5)
        self.save_config = list(0x00, 0x00)
 
    def get_acc(self):
        try:
            self.raw_acc_x = self.i2c.read_i2c_block_data(self.addr, 0x34, 2)
            self.raw_acc_y = self.i2c.read_i2c_block_data(self.addr, 0x35, 2)
            self.raw_acc_z = self.i2c.read_i2c_block_data(self.addr, 0x36, 2)
        except IOError:
            print("ReadError: gyro_acc")
            return (0, 0, 0)
        else:
            self.k_acc = 16 
 
            self.acc_x = (self.raw_acc_x[1] << 8 | self.raw_acc_x[0]) / 32768 * self.k_acc
            self.acc_y = (self.raw_acc_y[1] << 8 | self.raw_acc_y[0]) / 32768 * self.k_acc
            self.acc_z = (self.raw_acc_z[1] << 8 | self.raw_acc_z[0]) / 32768 * self.k_acc
            if self.acc_x >= self.k_acc:
                self.acc_x -= 2 * self.k_acc 
        
            if self.acc_y >= self.k_acc:
               self.acc_y -= 2 * self.k_acc
                
            if self.acc_z >= self.k_acc:
                self.acc_z -= 2 * self.k_acc
            return (self.acc_x, self.acc_y, self.acc_z)
    def get_gyro(self): # Angular velocities
        try:
            self.raw_gyro_x = self.i2c.read_i2c_block_data(self.addr, 0x37, 2)
            self.raw_gyro_y = self.i2c.read_i2c_block_data(self.addr, 0x38, 2)
            self.raw_gyro_z = self.i2c.read_i2c_block_data(self.addr, 0x39, 2)
        except IOError:
            print("ReadError: gyro_gyro")
            return (0, 0, 0)
        else:
            self.k_gyro = 2000
            self.gyro_x = (self.raw_gyro_x[1] << 8 | self.raw_gyro_x[0]) / 32768 * self.k_gyro
            self.gyro_y = (self.raw_gyro_y[1] << 8 | self.raw_gyro_y[0]) / 32768 * self.k_gyro
            self.gyro_z = (self.raw_gyro_z[1] << 8 | self.raw_gyro_z[0]) / 32768 * self.k_gyro
 
            if self.gyro_x >= self.k_gyro:
                self.gyro_x -= 2 * self.k_gyro
        
            if self.gyro_y >= self.k_gyro:
               self.gyro_y -= 2 * self.k_gyro
                
            if self.gyro_z >= self.k_gyro:
                self.gyro_z -= 2 * self.k_gyro
 
            return (self.gyro_x, self.gyro_y, self.gyro_z)
         
 
    def get_angle(self): #NED coordinate system | Z-Y-X Euler Rotation
        try:
            self.raw_angle_x = self.i2c.read_i2c_block_data(self.addr, 0x3d, 2)
            self.raw_angle_y = self.i2c.read_i2c_block_data(self.addr, 0x3e, 2)
            self.raw_angle_z = self.i2c.read_i2c_block_data(self.addr, 0x3f, 2)
        except IOError:
            print("ReadError: gyro_angle")
            return (0, 0, 0)
        else:
            self.k_angle = 180
 
            self.angle_x = (self.raw_angle_x[1] << 8 | self.raw_angle_x[0]) / 32768 * self.k_angle
            self.angle_y = (self.raw_angle_y[1] << 8 | self.raw_angle_y[0]) / 32768 * self.k_angle
            self.angle_z = (self.raw_angle_z[1] << 8 | self.raw_angle_z[0]) / 32768 * self.k_angle
            if self.angle_x >= self.k_angle:
                self.angle_x -= 2 * self.k_angle
        
            if self.angle_y >= self.k_angle:
                self.angle_y -= 2 * self.k_angle
                
            if self.angle_z >= self.k_angle:
                self.angle_z -= 2 * self.k_angle
            return (self.angle_x, self.angle_y, self.angle_z)
    
    def get_quaternion(self):
        try:
            self.raw_q0 = self.i2c.read_i2c_block_data(self.addr, 0x51, 2)
            self.raw_q1 = self.i2c.read_i2c_block_data(self.addr, 0x52, 2)
            self.raw_q2 = self.i2c.read_i2c_block_data(self.addr, 0x53, 2)
            self.raw_q3 = self.i2c.read_i2c_block_data(self.addr, 0x54, 2)
        except IOError:
            print("ReadError: gyro_angle(quat)")
            return (0, 0, 0, 0)
        else:
            self.q_w = (self.raw_q0[1] << 8 | self.raw_q0[0]) / 32768
            self.q_x = (self.raw_q1[1] << 8 | self.raw_q1[0]) / 32768
            self.q_y = (self.raw_q2[1] << 8 | self.raw_q2[0]) / 32768
            self.q_z = (self.raw_q3[1] << 8 | self.raw_q3[0]) / 32768

            return (self.q_w, self.q_x, self.q_y, self.q_z)
    
    def get_magnetic_readings(self):
        try:
            self.raw_mag_x = self.i2c.read_i2c_block_data(self.addr, 0x3a, 2)
            self.raw_mag_y = self.i2c.read_i2c_block_data(self.addr, 0x3b, 2)
            self.raw_mag_z = self.i2c.read_i2c_block_data(self.addr, 0x3c, 2)
        except IOError:
            print("ReadError: gyro_magnetic output")
            return (0, 0, 0)
        else:
            self.mag_x = (self.raw_mag_x[1] << 8 | self.raw_mag_x[0])
            self.mag_y = (self.raw_mag_y[1] << 8 | self.raw_mag_y[0])
            self.mag_z = (self.raw_mag_z[1] << 8 | self.raw_mag_z[0])
            
            return (self.mag_x, self.mag_y, self.mag_z)
    
    def get_temperature(self):
        try:
            self.raw_temp = self.i2c.read_i2c_block_data(self.addr, 0x40, 2)
        except IOError:
            print("ReadError: gyro_temperature")
            return 0
        else:
            self.temp = (self.raw_temp[1] << 8 | self.raw_temp[0]) / 100
            
            return self.temp
    
    '''Set the content of data returned by the device'''
    def set_content(self):
        try:
            ctnt = list(0x30, 0x02) # 0x02 -> activate quaternion output
            self.i2c.write_i2c_block_data(self.addr, 0x69, self.begin_config)
            self.i2c.write_i2c_block_data(self.addr, 0x02, ctnt)
            self.i2c.write_i2c_block_data(self.addr, 0x00, self.save_config)
            print("Quaternion content active!")
        except IOError:
            print("WriteError: gyro_content")
    
    '''Set the rate of data returned by the device: 0x08->50Hz | 0x09->100Hz | 0x0b->200Hz'''
    def set_rate(self, dta_frequency):
        try:
            ctnt = list(dta_frequency, 0x00)
            self.i2c.write_i2c_block_data(self.addr, 0x69, self.begin_config)
            self.i2c.write_i2c_block_data(self.addr, 0x03, ctnt)
            self.i2c.write_i2c_block_data(self.addr, 0x00, self.save_config)
            print("Data rate set | Restart device!")
        except IOError:
            print("WriteError: gyro_frequency")
    
    '''Set the device install direction: 0X00-> HORIZONTAL, 0X01-> VERTICAL'''
    def set_install_direction(self, direction):
        try:
            ctnt = list(direction, 0x00)
            self.i2c.write_i2c_block_data(self.addr, 0x69, self.begin_config)
            self.i2c.write_i2c_block_data(self.addr, 0x23, ctnt)
            self.i2c.write_i2c_block_data(self.addr, 0x00, self.save_config)
            print("Install direction set!")
        except IOError:
            print("WriteError: gyro_install_direction")
    
    '''Reset device configuration'''
    def reset(self):
        try:
            ctnt = list(0x01, 0x00)
            self.i2c.write_i2c_block_data(self.addr, 0x69, self.begin_config)
            self.i2c.write_i2c_block_data(self.addr, 0x00, ctnt)
            print("Device reset complete!")
        except IOError:
            print("WriteError: gyro_install_direction")
    
def main():
    head_gyro = Gyro(0x50)
    if(do_config):
        head_gyro.set_content() #Activate quaternion data
        head_gyro.set_install_direction(0x00) #Set install direction Horizontal
        head_gyro.set_rate(0x08) #Set data rate to 50Hz
        print("Device configuration completed! Change boolean and re-power device!")
        exit(0)
    else:
        rospy.init_node('wt901_node', anonymous=False)
        pub = rospy.Publisher('imu/data_raw', Imu, latch=False, queue_size=10)
        msg = Imu()
        msg.header.frame_id = "wt901_imu"
        r = rospy.Rate(50) # 50hz

        while(not rospy.is_shutdown):
            if(debug):
                print(" Acc:  " + repr(head_gyro.get_acc()) + "\n",
                      "Gyro: " + repr(head_gyro.get_gyro()) + "\n",
                      "Angle :" + repr(head_gyro.get_angle()) + "\n")
            msg.header.stamp = rospy.Time.now()
            quat = head_gyro.get_quaternion()
            acc = head_gyro.get_acc()
            gyro = head_gyro.get_gyro()

            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

            msg.linear_acceleration.x = acc[0]
            msg.linear_acceleration.y = acc[1]
            msg.linear_acceleration.z = acc[2]

            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            pub.publish(msg)
            r.sleep()

if __name__ == '__main__':
  main()
