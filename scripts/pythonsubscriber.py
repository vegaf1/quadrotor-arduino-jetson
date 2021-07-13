import time 
import zmq
import sys

from pwm_pb2 import *


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.bind("tcp://*:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

pwm = PWM()

while True: 

	message = socket.recv()
	pwm.ParseFromString(message)
	
	motor1 = pwm.motor1
	motor2 = pwm.motor2
	motor3 = pwm.motor3
	motor4 = pwm.motor4
	#print(type(motor1))
	print(str(pwm))
