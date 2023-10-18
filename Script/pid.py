import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation

timeAxis = []
index = 0
pidOutput = []

def draw(void):
    global index
    data = ser.readline().split()
    if(len(data) == 0):
        return
    index += 1
    if(len(timeAxis) > 100):
        timeAxis.pop(0)
        pidOutput.pop(0)
    timeAxis.append(index)
    pidOutput.append(float(data[0]))
    plt.cla()
    plt.plot(timeAxis,pidOutput)

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0)
    fig = plt.figure(figsize=(20, 10), dpi=100)
    ani = FuncAnimation(fig=fig,func=draw,interval=1,cache_frame_data=False)
    plt.show()
