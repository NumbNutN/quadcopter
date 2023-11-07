import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation


outputAxisDist = {
    'P':[],
    'M':[],
    # 'r':[],
    'G':[],
    'A':[],
    #'a':[],'b':[],'c':[],'d':[]
}

outputAxisInfo = {
    'P':{"index":1,"label":"pitch"},
    # 'p':{"index":2,"label":"pitch omega"},
    # 'r':{"index":3,"label":"roll alpha"},
    'R':{"index":2,"label":"roll"},
    # 'a':{"index":5,"label":"motor 1"},
    # 'b':{"index":6,"label":"motor 2"},
    # 'c':{"index":7,"label":"motor 2"},
    # 'd':{"index":8,"label":"motor 2"},
    'M':{"index":1,"label":"mix"},
    'A':{"index":2,"label":"accel"},
    'G':{"index":2,"label":"gyro"},
}

def draw(void):

    data = ser.readline().split()
    if(len(data) < 2):
        return
    
    label = data[0].decode("utf-8")
    if label not in outputAxisDist:
        return
    value = float(data[1])
    outputAxisDist[label].append(value)

    if(len(outputAxisDist[label]) > 100):
        outputAxisDist[label].pop(0)
    plt.subplot(1,2,outputAxisInfo[label]['index'])
    plt.cla()
    plt.title(outputAxisInfo[label]['label'],loc='left')
    plt.plot(range(len(outputAxisDist[label])),outputAxisDist[label])

if __name__ == "__main__":
    ser = serial.Serial('COM13',115200,timeout=0)

    fig = plt.figure(figsize=(20, 10), dpi=100)

    ani = FuncAnimation(fig=fig,func=draw,interval=1,cache_frame_data=False)
    plt.show()
