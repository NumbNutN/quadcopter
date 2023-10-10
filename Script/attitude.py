import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation

def quat2DCM(quat:list) -> list:
    a:float=quat[0]
    b:float=quat[1]
    c:float=quat[2]
    d:float=quat[3]
    return [[1-2*c*c-2*d*d,2*b*c-2*a*d,2*a*c+2*b*d],
         [2*b*c+2*a*d,-2*b*b-2*d*d,2*c*d-2*a*b],
         [2*b*d-2*a*c,2*a*b+2*c*d,1-2*b*b-2*c*c]]
    

def quat2EulerAngle_zyx(quat:list) -> np.ndarray:
    a:float=quat[0]
    b:float=quat[1]
    c:float=quat[2]
    d:float=quat[3]
    return np.array(
        -math.asin(2*b*d-2*a*c),
        math.atan((2*a*b + 2*c*d) / (1-2*b*b - 2*c*c)),
        math.atan((2*b*c + 2*a*d) / (1-2*c*c - 2*d*d))
    )

if __name__ == "__main__":
    matplotlib.use('TkAgg')
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    while(1):
        data = ser.readline().split()
        if(len(data)!=4):
            continue
        # quaternion repesenting current attitude
        quat = [float(x) for x in data]

        rotation_matrix = quat2DCM(quat)
        
        rotation_matrix[0].append(rotation_matrix[0][0])
        rotation_matrix[1].append(rotation_matrix[1][0])
        rotation_matrix[2].append(rotation_matrix[2][0])
        rotation_matrix[0].append(0)
        rotation_matrix[1].append(0)
        rotation_matrix[2].append(0)
        rotation_matrix[0].append(rotation_matrix[0][1])
        rotation_matrix[1].append(rotation_matrix[1][1])
        rotation_matrix[2].append(rotation_matrix[2][1])
        rotation_matrix[0].append(0)
        rotation_matrix[1].append(0)
        rotation_matrix[2].append(0)
        rotation_matrix[0].append(rotation_matrix[0][2])
        rotation_matrix[1].append(rotation_matrix[1][2])
        rotation_matrix[2].append(rotation_matrix[2][2])        

        rotation_matrix = np.array(rotation_matrix)
        
        ax.plot(rotation_matrix[0],rotation_matrix[1],rotation_matrix[2],color='m')
        
        plt.show()