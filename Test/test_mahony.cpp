#include "config.h"

#if TEST_MAHONY_EN > 0u

#include <iostream>

#include "gradient_decent.hpp"
#include "hmc_5583l.hpp"
#include "mpu6050.hpp"
#include "quaternion.hpp"
#include "quat_math.hpp"

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
  // 初始姿态
  get_origin_attitude();
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
          omega_measure =
              accel ^ (attitude.conjugate() * earth_accel * attitude);
          quaternion last_gyro = mpu6050_ptr->get_last_gyro();
          integral_omega_measure += ki * omega_measure * deltaT;

          old_omega_measure = omega_measure;
          return 0.5 * q *
                 (last_gyro + integral_omega_measure + kp * omega_measure);
        }).normalization();
    old_attitude = attitude;
    OSTimeDlyHMSM(0, 0, 0, 2);
  }
}

void TEST_Mahony_Init() {
  OS_ERR err;
  OSTaskCreate(TEST_Mahony, NULL, &Stk_Mahony[511], TASK_MAHONY_READ_PRIO);
  OSTaskNameSet(TASK_MAHONY_READ_PRIO, (INT8U *)"MAHANY", &err);
}
#endif
