import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation

def draw_vector(void):
        data = ser.readline().split()
        if(len(data)!=4):
            return
        # quaternion repesenting current attitude
        quat = [float(x) for x in data]
        ax.scatter(quat[1],quat[2],quat[3],color='m')

if __name__ == "__main__":
    matplotlib.use('TkAgg')
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    
    ani = FuncAnimation(fig=fig,func=draw_vector,interval=1,cache_frame_data=False)
        
    plt.show()