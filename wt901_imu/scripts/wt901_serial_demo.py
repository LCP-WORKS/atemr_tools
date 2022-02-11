#! /usr/bin/env python3

import serial, struct

g = 9.81
conv = 16.0 / 32768.0  * g

ser = serial.Serial('/dev/wt901IMU',115200)

def run():
	while(True):
		data = ser.read(11)
		#print('Data: ', (data[1] - 30))
		if ((data[1]-30) == 51):
			axes = struct.unpack("<hhh", data[2:8])
			x, y, z = [a*conv for a in axes]
			print(x, ":", y, ":", z)

if __name__ == '__main__':
	run()
	ser.close()
