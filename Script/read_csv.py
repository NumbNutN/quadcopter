import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csv_file = "WAVE(2023.11.6-21.37.26).csv"
df = pd.read_csv(csv_file,encoding='utf-8')
x = df[['IntOutPitch']]
y = df[['AngularAccel']]

# 最小二乘法
def linear_regression(x:np.ndarray,y:np.ndarray):
    N = len(x)
    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum(x**2)
    sumxy = sum(x*y)
    A = np.mat([[N,sumx[0]],[sumx[0],sumx2[0]]])
    b = np.array([sumy,sumxy])
    return np.linalg.solve(A,b)

b,a = linear_regression(x.values,y.values)
print(a,b)
fig = plt.figure(figsize=(20, 10), dpi=100)
plt.xlabel("PWM signal cycle duty")
plt.ylabel("angular accelation in pitch")

x_hat = np.arange(-0.05,0.05,0.001)
y_hat = b+a*x_hat
plt.plot(x_hat,y_hat)
plt.scatter(x.values,y.values)
plt.show()