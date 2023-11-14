import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation
import matplotlib.style as mplstyle


def correct(dat):
    #校验数据完整性 x y z sum
    return abs(dat[0] + dat[1] + dat[2] - dat[3]) < 0.1

def param_constructor(vec:list):
    #构建一条线性方程式的参数
    return [vec[1]**2,vec[2]**2,vec[0],-vec[1],-vec[2],1]

def least_square(mat:list,res:list):
    #最小二乘法计算
    mat = np.matrix(mat)
    res = np.array(res)

    #debug
    A_T_A_N1_N1 = (mat.T  * mat).I
    funcL = A_T_A_N1_N1*mat.T
    return np.dot(funcL,res)

def parse(res:np.ndarray):
    res = np.array(res)
    res = res[0]
    ox = -res[2] / 2
    oy = -res[3] / (2*res[0])
    oz = -res[4] / (2*res[1])
    rx = math.sqrt(ox**2 + res[0]*oy**2 + res[1]*oz**2 - res[5])
    ry = math.sqrt(rx**2 / res[0])
    rz = math.sqrt(rx**2 / res[1])
    return [ox,oy,oz,rx,ry,rz]

#callback 采集数据打印在3D坐标系中
cnt = 0
paramLst = []
resLst = []
def draw_vector(void):
        global cnt
        data = ser.readline().split()
        if(len(data)!=4):
            return
        print(data)
        vec = [float(x) for x in data]
        if(correct(vec)):
            paramLst.append(param_constructor(vec))
            resLst.append(-vec[0]**2)
            ax.scatter(vec[0],vec[1],vec[2],color='m')
            cnt += 1
        if cnt >= 200:
            res:np.ndarray = least_square(paramLst,resLst)
            print(res)
            res = parse(res)
            print(res)
            exit()

if __name__ == "__main__":
    #matplotlib.use('TkAgg')
    ser = serial.Serial('COM13',9600,timeout=0)
    mplstyle.use('fast')
    #matplotlib.use('agg')
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    
    ani = FuncAnimation(fig=fig,func=draw_vector,interval=1,cache_frame_data=False)
        
    plt.show()