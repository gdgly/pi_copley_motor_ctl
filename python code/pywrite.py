#!/usr/bin/env python3
import zmq
import sys, os
import serial

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:8033")

serialName = 'COM12'

nread = ''
with serial.Serial(serialName, 115200, timeout=1) as ser:
    if ser.open:
        while True:
            t = ser.read(1).decode()
            if t != '\n' and t != '':
                 nread = nread.__add__(t)
            else:
                print(nread )
                socket.send_string(nread)
                nread = ''
                print('\n')
 #               break

    ser.close


