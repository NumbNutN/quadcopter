#include "config.h"

#if TEST_MAHONY_EN > 0u

#include <iostream>

#include "gradient_decent.hpp"
#include "hmc_5583l.hpp"
#include "mpu6050.hpp"
#include "quaternion.hpp"
#include "quat_math.hpp"
#include "anotc.hpp"

#include <math.h>

#include "delay.h"
#include "os.h"
#include "test_tasks.h"

#define BETA 0.033
using namespace std;
OS_STK Stk_Mahony[1024];

quaternion earth_magnetic{0, 0.66436113595878288, 0, 0.74741172122703259};
// quaternion earth_magnetic{0,1,0,0};
mpu6050 *mpu6050_ptr;
hmc_558l *hmc_ptr;
quaternion attitude_gyro;
quaternion acceleration_err_fun_gradient;
quaternion magnetic_err_func_gradient;
quaternion attitude_mixed;
float deltaT;
quaternion attitude;
quaternion old_attitude = {1, 0, 0, 0};
quaternion earth_accel = {0, 0, 0, 1};

/* Madgrick */
quaternion accelator_gradient(const quaternion &q,
                              const quaternion &acceleration) {
  quaternion func_g{2 * (q[1] * q[3] - q[0] * q[2]) - acceleration[1],
                    2 * (q[0] * q[1] + q[2] * q[3]) - acceleration[2],
                    1 - 2 * q[1] * q[1] - 2 * q[2] * q[2] - acceleration[3]};
  return quaternion{-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]} * func_g[0] +
         quaternion{2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]} * func_g[1] +
         quaternion{0, -4 * q[1], -4 * q[2], 0} * func_g[2];
}

quaternion get_origin_attitude() {
  double c = 1e-6;
  double s = 0;
  double step = 0.5;
  // adagrad自适应步长
  // 获取加速度
  quaternion accel = mpu6050_ptr->get_acceleration();
  size_t i = 10;
  do {
    quaternion accel_grad = accelator_gradient(attitude, accel);
    s += accel_grad & accel_grad;
    attitude = attitude - step / sqrt(s + c) * accel_grad;
  } while (--i);
}

/* 惯性传感器数据 */
#define ANO_MAG_CROSS_FMT 0xF4
#define ANO_MAG_CROSS_DATLEN 0x04
auto mag_cross_frame = anotcDataFrame<ANO_MAG_CROSS_DATLEN>(ANO_MAG_CROSS_FMT);
void TEST_Mahony(void *arg) {
  mpu6050 mpu6050_dev;
  hmc_558l hmc_dev;
  quaternion gyroData;
  mpu6050_ptr = &mpu6050_dev;
  hmc_ptr = &hmc_dev;
  // 积分项 是误差的累计
  static quaternion integral_omega_measure{0, 0, 0, 0};
  static quaternion old_omega_measure;
  // //校准陀螺仪的零偏误差
  mpu6050_dev.alignment(100);

  //地磁计自测
  //hmc_dev.test();
  
  hmc_dev.measure_earth_magnetic(100);
  // 初始姿态
  get_origin_attitude();
  //获得大地地磁 TEST0
  const static quaternion b_measured = hmc_ptr->get_measured_earth_magnetic().normalization();
  const static quaternion b_E = ((attitude * b_measured * attitude.conjugate()).no(3)).normalization();

  for (;;) {
    mpu6050_dev.update();
    // hmc_dev.update();
    deltaT = mpu6050_dev.getSamplePeriod();
    // 一阶龙格图塔计算姿态
    attitude =
        RK1(0, old_attitude, deltaT, [](double t, quaternion q) -> quaternion {
          float ki = 0.05;
          float kp = 1.5;
          
          // 反应测量加速度和估计加速度的误差
          quaternion omega_measure;
          quaternion accel = mpu6050_ptr->get_acceleration().normalization();

          quaternion m_b = hmc_ptr->getMagnetic().no(3).normalization(); /* 获取磁力计观测值 */
          //quaternion b_E = {0,0,(float)sqrt((double)(m_b[1]*m_b[1] + m_b[2]*m_b[2])),m_b[3]}; /* 作为当前地理磁场 */
          
          quaternion accel_cross = accel ^ (attitude.conjugate() * earth_accel * attitude);
          //TEST 1 加入地磁
          quaternion mag_cross = m_b ^ (attitude.conjugate() * b_E * attitude);
          omega_measure =
            accel_cross;
//            0.5 *mag_cross;
#if TEST_PRINT_EN > 0u
          // static size_t cnt = 0;
          // ++cnt;
          // if(cnt == 50){
          //   cout << "cro " << mag_cross << endl;
          //   cnt = 0;
          // }
#else     
          //TEST2 打印叉乘
          //Anotc Uplink协议打印地磁绝对值和测量值的叉乘
          // static size_t cnt = 0;
          // ++cnt;
          // if(cnt == 25){
          //   auto dat = mag_cross.length();
          //   mag_cross_frame.pack(&dat,4);
          //   mag_cross_frame.send();
          //   cnt = 0;
          // }
#endif
          // TEST3计算磁力计的数据乘以姿态矩阵的结果
          //quaternion earth_mag = attitude * hmc_ptr->getMagnetic().normalization() * attitude.conjugate();
          
          quaternion last_gyro = mpu6050_ptr->get_last_gyro();
          integral_omega_measure += ki * omega_measure * deltaT;

          old_omega_measure = omega_measure;
          return 0.5 * q *
                 (last_gyro + integral_omega_measure + kp * omega_measure);
        }).normalization();
    old_attitude = attitude;
    OSTimeDlyHMSM(0, 0, 0, 10);
  }
}

void TEST_Mahony_Init() {
  OS_ERR err;
  OSTaskCreate(TEST_Mahony, NULL, &Stk_Mahony[511], TASK_MAHONY_READ_PRIO);
  OSTaskNameSet(TASK_MAHONY_READ_PRIO, (INT8U *)"MAHANY", &err);
}
#endif
