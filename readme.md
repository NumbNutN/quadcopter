## 四旋翼飞行器



一个四旋翼飞行器项目飞控设计仓库，基于IMU测算机体姿态并使用反馈控制算法控制四枚无刷电机的动能分配实现飞行姿态稳定受控



#### 物料表

| 元器件           | 用途               | 通信方式                | 附加信息 |
| ---------------- | ------------------ | ----------------------- | -------- |
| STM32F401RET6    | 微控制器           | -                       | -        |
| MPU6050          | 惯性测量单元       | I2C                     | -        |
| HMC5881          | 地磁场测量单元     | I2C                     | -        |
| ECS              | 无刷电机控速       | PWM 1nm-2nm pulse width | -        |
| Receiver         | 遥控器控制信号接收 | PPM                     | -        |
| 串口协议输出模块 | 姿态数据输出       | USART                   | optional |
| OLED             | 日志输出           | I2C                     | optional |



#### 项目构建和工具链要求

+ [newlib](https://sourceware.org/newlib/)
+ [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm)



#### 项目构建

```
make

#ELF Format: build/quadcopter.elf
#Binary:     build/quadcopter.bin
#Hex File:   build/quadcopter.hex
```



#### 管脚分配



<img src="https://raw.githubusercontent.com/NumbNutN/NumbNutN.github.io/main/img/post-img/2023-11-08-quadcopter-pinout-distribution.png" alt="image-20231107211127305" style="zoom:50%;" />

| 外设           | 复用功能  | 引脚 |
| -------------- | --------- | ---- |
| ECS1 信号线    | TIM3_CH1  | PC6  |
| ECS2 信号线    | TIM3_CH2  | PC7  |
| ECS3 信号线    | TIM3_CH3  | PC8  |
| ECS4 信号线    | TIM3_CH4  | PC9  |
| I2C BUS SCL    | Default   | PB8  |
| I2C BUS SDA    | Default   | PB9  |
| USART TX       | USART1_TX | PA9  |
| USART RX       | USART1_RX | PA10 |
| 接收机输入信号 | TIM1_CH1  | PA8  |



#### 硬件参考方案

<img src="https://raw.githubusercontent.com/NumbNutN/NumbNutN.github.io/main/img/post-img/2023-11-08-quadcopter-reference.png" style="zoom: 50%;" />

#### Warning

处于早期测试阶段，具有较大安全隐患，仅可将飞行器约束在稳固的刚性物体的状态下调试开发

