import sys
import zmq
import re
import collections
import numpy as np
import time
import datetime

data_path = 'D:\\ubuntu_gx\\pytest\\motor_log_2018_03_21_1750.txt'

context8033 = zmq.Context()
socket8033 = context8033.socket(zmq.PUB)
socket8033.bind("tcp://*:8033")

context8034 = zmq.Context()
socket8034 = context8034.socket(zmq.PUB)
socket8034.bind("tcp://*:8034")



f = open(data_path)

lines = f.readlines()

ln = len(lines)
print(ln)
cnt = 1
msg = []


while cnt < ln:
    temp = re.sub("time=", "", lines[cnt])
    temp = re.sub("force=", "", temp)
    temp = re.sub("position=", "", temp)
    temp = re.sub("state=", "", temp)
    temp = re.sub("speed_cmd=", "", temp)
    temp = re.sub("p1=", "", temp)
    temp = re.sub("p2=", "", temp)
    temp = re.sub("p3=", "", temp)
    temp = re.sub("\n", "", temp)
    temp = temp.split(' ')
    print("force={}".format(temp[1]))
    socket8034.send_string("force={}".format(temp[1]))
    nwrite = "time={} p1={} p2={} p3={}".format(temp[0], temp[5], temp[6], temp[7])
    socket8033.send_string(nwrite)
    print(nwrite)
    if temp != ['']:
        msg.append(temp)
    cnt += 1
    if cnt == ln - 100:
        cnt = 100

    time.sleep(0.008)

