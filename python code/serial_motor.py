import sys
import os
import serial
import zmq
import time
import re


def motor_ctrl(ser, str):
    "this is a test py module"
    nread = ''
    nwrite = str.encode()
    ser.write(nwrite)
    while True:
        t = ser.read(1).decode()
        if t != '\r' and t != '':
            nread = nread.__add__(t)
            if len(nread) > 1024:
                print("rev too much data and can not find the end of messages")
                return "no data"
        else:
            return nread


# context = zmq.Context()
# socket = context.socket(zmq.REP)
# socket.bind("tcp://*:8041")

# serialpath = "COM3"
serialpath = "/dev/ttyUSBmotor"



with serial.Serial(serialpath, 9600, timeout=1) as ser:
    if ser.open:
        nwrite = "s r0x90 115200\n"
        ser.write(nwrite.encode())
        time.sleep(0.1)
        ser.close()

time.sleep(0.1)

with serial.Serial(serialpath, 115200, timeout=1) as ser:
    if ser.open:
        nwrite = "s r0x24 0\n"
        ser.write(nwrite.encode())
        time.sleep(0.1)
        ser.close()

time.sleep(0.1)

#os.system('~/work/motor_pi/./motor_replay.sh')





# with serial.Serial(serialpath, 115200, timeout=1) as ser:
#     try:
#         if ser.open:
#             while True:
#                 nwrite = socket.recv_string()
#                 print(nwrite)
#                 nrev = motor_ctrl(ser, nwrite)
#                 print(nrev)
#                 socket.send_string("{}".format(nrev))
#
#
#     except KeyboardInterrupt:
#         ser.close()
#         print("motor communication abort please restart motor_driver")


