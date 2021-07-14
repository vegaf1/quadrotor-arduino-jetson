import time 
import zmq
import sys
import serial 
import struct

from pwm_pb2 import *


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.bind("tcp://*:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

arduino = serial.Serial(port = "/dev/ttyACM0", baudrate = 115200)

pwm = PWM()

#global armed
armed = 0

#time.sleep(10)
def calibrationcheck():
	global armed
	armed_key = 'armed'
	calibrated_key = 'calibrated'
	while armed==0:
		line = arduino.readline()
		update = line.decode()
		numupdate = int(update)
		print(numupdate)
		print(type(update))
		print(armed)
		if (numupdate == 2):
			armed = 1
			print("RUNNING MOTORS")
			

def running():

	while armed == 1: 

		message = socket.recv()
		pwm.ParseFromString(message)
		
		motor1 = pwm.motor1
		motor2 = pwm.motor2
		motor3 = pwm.motor3
		motor4 = pwm.motor4
		#print(type(motor1))
		#motorstring = struct.pack('>IIII', int(motor1), int(motor2), int(motor3), int(motor4))
		#print(motorstring.decode("utf-8"))
		#arduino.write(struct.pack('>llll', int(motor1), int(motor2), int(motor3), int(motor4)))
		arduino.write(struct.pack('>BBBB', int(motor1/10), int(motor2/10),int(motor3/10), int(motor4/10)))
		line = arduino.readline()
		voltstring = line.decode()
		voltnum = float(voltstring)
		print(voltnum)
		print(str(pwm))


def main():
	
	calibrationcheck()

	running()

if __name__ == "__main__":
	main()
	
