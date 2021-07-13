#!/usr/bin/env julia

using ZMQ
using ProtoBuf
using Serialization

include("/home/fausto/protobuff/juliaout/pwm_pb.jl")
include("/home/fausto/protobuff/juliaout/pwm.jl")

context = Context()
socket = Socket(context, PUB)
connect(socket, "tcp://localhost:5555")
iob = IOBuffer()

while true
    for i in 1200:100:1500
    writeproto(iob, PWM(; motor1 = i, motor2 = i+100, motor3 = i+200, motor4 = i+300))
    #msg = serialize(PWM)
    ZMQ.send(socket, Message(iob))
    yield()
    #data = readproto(iob, PWM())
    #print(PWM(iob))
    #print(data)
    print(Message(iob))
    print("sending")
    sleep(1)
    end 

    
end

close(socket)
close(context)
