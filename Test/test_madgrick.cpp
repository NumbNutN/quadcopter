#include "config.h"
#if TEST_MADGRICK_EN > 0u
#include "delay.h"
#include "gradient_decent.hpp"
#include "hmc_5583l.hpp"
#include "mpu6050.hpp"
#include "quat_math.hpp"
#include "os.h"
#include "quaternion.hpp"
#include "test_tasks.h"
#include <iostream>
#include <math.h>
#define BETA 0.033
using namespace std;
OS_STK Stk_Mahony[512];

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

quaternion magnetic_gradient(const quaternion &q, const quaternion& earth_mag,const quaternion &magnetic) {
  quaternion func_m{
      0,
      (float)(2 * earth_mag[1] * (0.5 - q[2] * q[2] - q[3] * q[3]) +
          2 * earth_mag[3] * (q[1] * q[3] - q[0] * q[2]) - magnetic[1]),
      (float)(2 * earth_mag[1] * (q[1] * q[2] - q[0] * q[3]) +
          2 * earth_mag[3] * (q[0] * q[1] + q[2] * q[3]) - magnetic[2]),
      (float)(2 * earth_mag[1] * (q[0] * q[2] + q[1] * q[3]) +
          2 * earth_mag[3] * (0.5 - q[1] * q[1] - q[2] * q[2]) - magnetic[3])};
  return quaternion{
             -2 * earth_mag[3] * q[2], 2 * earth_mag[3] * q[3],
             -4 * earth_mag[1] * q[2] - 2 * earth_mag[3] * q[0],
             -4 * earth_mag[1] * q[3] + 2 * earth_mag[3] * q[1]} *
             func_m[0] +
         quaternion{
             -2 * earth_mag[1] * q[3] + 2 * earth_mag[3] * q[1],
             2 * earth_mag[1] * q[2] + 2 * earth_mag[3] * q[0],
             2 * earth_mag[1] * q[1] + 2 * earth_mag[3] * q[3],
             -2 * earth_mag[1] * q[0] + 2 * earth_mag[3] * q[2]} *
             func_m[1] +
         quaternion{2 * earth_mag[1] * q[2],
                    2 * earth_mag[1] * q[3] - 4 * earth_mag[3] * q[1],
                    2 * earth_mag[1] * q[0] - 4 * earth_mag[3] * q[2],
                    2 * earth_mag[1] * q[1]} *
             func_m[2];
}
quaternion GuassianFilter(const quaternion &gyro) {
  static quaternion historyList[5];
  static size_t index = 0;
  quaternion average = {0, 0, 0, 0};
  historyList[index++ % 5] = gyro;
  for (int i = 0; i < 5; ++i)
    average += historyList[i];
  // static vector<quaternion> v5(5,quaternion{0,0,0,0});
  return index >= 5 ? average / 5 : average / (index + 1);
}
quaternion acceratorGuassianFilter(const quaternion &accel) {
  static quaternion historyList[15];
  static size_t index = 0;
  quaternion average = {0, 0, 0, 0};
  if (!(index % 3)) {
    historyList[index / 3 % 15] = accel;
  }
  index++;
  for (int i = 0; i < 15; ++i)
    average += historyList[i];
  return index >= 15 ? average / 15 : average / (index + 1);
}
mpu6050 *mpu6050_ptr;
hmc_558l *hmc_ptr;
quaternion attitude_gyro;
quaternion acceleration_err_fun_gradient;
quaternion magnetic_err_func_gradient;
quaternion attitude_mixed;
float deltaT;
quaternion attitude;
quaternion old_attitude = {1, 0, 0, 0};
void TEST_Madgwick(void *arg) {
  mpu6050 mpu6050_dev;
  hmc_558l hmc_dev;
  quaternion gyroData;
  mpu6050_ptr = &mpu6050_dev;
  hmc_ptr = &hmc_dev;
  // //校准陀螺仪的零偏误差
  mpu6050_dev.alignment(100);
  for (;;) {
    mpu6050_dev.update();
    hmc_dev.update();
    deltaT = mpu6050_dev.getSamplePeriod();
    // 一阶龙格图塔计算姿态
    attitude_gyro =
        RK1(0, old_attitude, deltaT, [](double t, quaternion q) -> quaternion {
          quaternion last_gyro = mpu6050_ptr->get_last_gyro();

          return 0.5 * q *
                 ((t / deltaT) * mpu6050_ptr->get_current_gyro() +
                  (1 - t / deltaT) * last_gyro);
        });

    // 为加速度计的误差函数计算梯度
    acceleration_err_fun_gradient = accelator_gradient(
        old_attitude, acceratorGuassianFilter(mpu6050_dev.get_acceleration()));
    
    // 为磁力计的误差函数计算梯度
    quaternion m_b = hmc_dev.getMagnetic(); /* 获取磁力计观测值 */
    quaternion b_E = {0,(float)sqrt((double)(m_b[1]*m_b[1] + m_b[2]*m_b[2])),0,m_b[3]}; /* 作为当前地理磁场 */
    magnetic_err_func_gradient = magnetic_gradient(old_attitude,b_E,m_b);

    attitude_mixed =
    (acceleration_err_fun_gradient+magnetic_err_func_gradient);
    // 不包含地磁
    attitude = (attitude_gyro -
                0.08 * attitude_mixed.normalization() * deltaT)
                   .normalization();
    old_attitude = attitude;
    OSTimeDlyHMSM(0, 0, 0, 4);
  }
}

void TEST_Madgwick_Init() {

  OSTaskCreate(TEST_Madgwick, NULL, &Stk_Mahony[511], TASK_MADGWICK_READ_PRIO);
}

#endif