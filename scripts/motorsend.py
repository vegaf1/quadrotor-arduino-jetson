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

while True: 

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
	print(str(pwm))
