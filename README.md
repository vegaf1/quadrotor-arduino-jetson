# quadrotor-arduino-jetson

# Getting started 

1. Wire up the bread board with the arduino and motor ESC's as shown in the wiring diagram. 
   - Resistor Values: 100k (brown,black,yellow), 20k (red, black, orange), 2.2k (red, red, red)
   - Solder an additional line from the power distribution board which is connected to the battery and connect it to the breadboard as shown in the diagram. 
   - Votage Divider will divide voltage to a maximum of 2.8 V which is read via the A1 pin on the Arduino. 

2. The quadrotor will use julia as a controller which provides control inputs to a python program that sends it to the Arduino via serial. Protocol buffers are used to serialize the messages between Julia and Python, and a packed structure was used to send it from the python script to the arduino. This program shows a simple demo of sending different pwm signals to the motors through this frame work. 

3. The Arduino code includes the calibration of the ESC's which is initialized everytime the arduino is plugged in.

4. The pwm.proto file contains the message that is passed between Julia and Python. 

# Running the programs

- julia juliapublisher.jl

- python3 motorsend.py

A publisher subscriber framework is used in Zero MQ to communicate between python and Julia. 

