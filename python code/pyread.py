#!/usr/bin/env python3
import sys, os

import numpy as np

import matplotlib.pyplot as plt

import pandas as pd

import re

data_path = "/home/shiki/ubuntu_gx/pytest/"

txt_name = "2018_2_26_1956.txt"
if len(sys.argv) > 1:
    txt_name = sys.argv[1]

msg = []

real_path = data_path + txt_name

f = open(real_path)

lines = f.readlines()
ln = len(lines)
print(ln)
cnt = 1
while cnt < ln-1:
    temp = re.sub("time=", "", lines[cnt])
    temp = re.sub("force=", "", temp)
    temp = re.sub("position=", "", temp)
    temp = re.sub("\n", "", temp)
    temp = temp.split(' ')
    if temp != ['']:
        msg.append(temp)
    cnt += 1

print(msg)
msg = np.array(msg, dtype=np.uint64)
print(msg[:,0])
print(msg[:,1])
print(msg[:,2])
plt.plot(msg[:,0], msg[:,1]*100, "g-", label="force")
plt.plot(msg[:,0], msg[:,2], "r-", label="position")

#plt.axis(1, 3000, 1, 3000)
plt.xlabel("time")
plt.ylabel("force/positon")
plt.title("test plot")
plt.grid(True)
plt.legend()
plt.show()

