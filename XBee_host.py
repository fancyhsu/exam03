import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import struct

# XBee setting
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())
s.write("ATMY 0x241\r\n".encode())
char = s.read(3)
print("Set MY <BASE_MY>.")
print(char.decode())
s.write("ATDL 0x240\r\n".encode())
char = s.read(3)
print("Set DL <BASE_DL>.")
print(char.decode())
s.write("ATID 0x0\r\n".encode())
char = s.read(3)
print("Set PAN ID <PAN_ID>.")
print(char.decode())
s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())
s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())
s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())
s.write("ATCN\r\n".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())
print("start sending RPC")

samples = 500
x = np.zeros(samples)
t = np.arange(1,samples+1)

i = 0
while True:
    s.write("/rpc_call/run\r".encode())
    line = s.readline()
    line = line.decode()
    
    if i != 0:
        x[i-1] = float(line)
        print(x[i-1])
        i += 1
    if i == 0 and line[0] == '0': i+=1
    time.sleep(1)
    
s.close()