import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import io
import struct


plt.subplot(2,1,1).axis([0, 129, 0, 255])
plt.ion()
plt.show()
plt.subplot(2,1,1).autoscale(enable = False)
plt.subplot(2,1,2).autoscale(enable = False)

plt.axis([0, 129, 0, 255])

results = []

port = "COM5"
ser = serial.Serial(port, 115200, timeout=0)

line = []
line_raw = []
line_derivative = []
x = range(0, 128)

delta = 20
threshold = 20

throwout = 20

while True:
    """
    ser.write(b"E")
    
    while len(line_raw) < 128:
        if not (ser.inWaiting() == 0):
            value = struct.unpack('B', ser.read())[0]
            line_raw.append(value)
   """ 


    ser.write(b"E")
    while len(line) < 128:
        if not (ser.inWaiting() == 0):
            value = struct.unpack('B', ser.read())[0]
            line.append(value)

    line = line[::-1]
    current_pos = -1
    output = -1
    min1 = -1

    minn = -1
    maxx = -1

    ser.write(b"E")
    while(current_pos == -1):
        if not (ser.inWaiting() == 0):
            current_pos = struct.unpack('B', ser.read())[0];
    while(output == -1):
        if not (ser.inWaiting() == 0):
            output = struct.unpack('B', ser.read())[0];

    while(minn == -1):
        if not (ser.inWaiting() == 0):
            minn = struct.unpack('B', ser.read())[0];

    while(maxx == -1):
        if not (ser.inWaiting() == 0):
            maxx = struct.unpack('B', ser.read())[0];

    print(128 - output, 128 - current_pos, 128 - minn, 128 - maxx)
    
    for i in range(0, int(throwout/2)+1):
        line_derivative.append(0)
    
    for i in range(int(throwout/2) + 1 ,128 - int(throwout/2)-1 ):
        line_derivative.append((line[i] -line[i-1]))
    
    for i in range(0, int(throwout/2)+1):
        line_derivative.append(0)

    
   

    """
    max1=0
    max2 =0
    max1Value=0
    max2Value = 0

    min1= 0
    min2 = 0
    min1Value=0
    min2Value = 0

    for d in range(len(line_derivative)):
        if abs(line_derivative[d]) > threshold:
            if line_derivative[d] > 0:
                if min1 != 0 and min2 == 0:
                    if max1Value < line_derivative[d]:
                        max1Value = line_derivative[d]
                        max1 = d
                    
                elif min2 != 0:
                    if max2Value < line_derivative[d]:
                        max2Vale = line_derivative[d]
                        max2 = d
            else:
                if max1 == 0:
                    if min1Value > line_derivative[d]:
                        min1Value = line_derivative[d]
                        min1 = d   
                elif max1 != 0:
                    if min2Value > line_derivative[d]:
                        min2Value = line_derivative[d]
                        min2 = d 

    line1 = (max1 + min1)/2

    line2 = (max2 + min2)/2
    

    print("Min1:", min1, "Max1", max1,"Min2:", min2, "Max2", max2, )
    """


    #line = data.decode('UTF-8').split(",")
    plt.clf()
    #plt.scatter(x, line_raw, c = 'b')

    plt.subplot(2,1,1).scatter(x, line, c = 'r')
    plt.subplot(2,1,2).scatter(x, line_derivative)
    plt.subplot(2,1,1).scatter(128 - current_pos, 10, c='g')
    plt.subplot(2,1,1).scatter(128 - minn, 10, c='c')
    plt.subplot(2,1,1).scatter(128 - maxx, 10, c='y')
    plt.subplot(2,1,2).axis([0,129,-50,50])
    plt.subplot(2,1,1).axis([0, 129, 0, 255])
    plt.draw()  
    plt.pause(0.0001)

    line = []
    line_raw = []
    line_derivative = []


ser.close()




