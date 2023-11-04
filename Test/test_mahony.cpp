#include "config.h"

#if TEST_MAHONY_EN > 0u

#include <iostream>

#include <math.h>

#include "gradient_decent.hpp"
#include "hmc_5583l.hpp"
#include "mpu6050.hpp"
#include "myMath.hpp"
#include "quaternion.hpp"
// #include "anotc.hpp"

#include "delay.h"
#include "os.h"
#include "test_tasks.h"

#define BETA 0.033

using namespace std;

OS_STK Stk_Mahony[1024];
OS_STK Stk_PrintMahony[1024];

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
    ano_send_dataFrame(attitude);
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
    hmc_dev.update();
    deltaT = mpu6050_dev.getSamplePeriod();

    // 一阶龙格图塔计算姿态
    attitude =
        RK1(0, old_attitude, deltaT, [](double t, quaternion q) -> quaternion {
          float ki = 0.2;
          float kp = 0.5;
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
    OSTimeDlyHMSM(0, 0, 0, 20);
  }
}

#define ANO_ATTITUDE_EULERANGLE_FMT 0x03
#define ANO_ATTITUDE_ENLERANGLE_DATLEN 0x07

void TEST_Task_Print_Attitude(void *arg) {

  // auto frame =
  // anotcDataFrame<ANO_ATTITUDE_ENLERANGLE_DATLEN>(ANO_ATTITUDE_EULERANGLE_FMT);
  int8_t data[7];
  for (;;) {
    ano_send_dataFrame(attitude);
    // int16_t roll = quat_get_Roll(attitude) * 100 * 57.3;
    // int16_t pitch = quat_get_Pitch(attitude) * 100 * 57.3;
    // int16_t yaw = quat_get_Yaw(attitude) * 100 * 57.3;
    // *(int16_t*)data = roll;
    // *(int16_t*)(data+2) = pitch;
    // *(int16_t*)(data+4) = yaw;
    // *(data+6)=0x01;

    // frame.pack(data);
    // frame.send();
    OSTimeDlyHMSM(0, 0, 0, 100);
  }
}

void TEST_Mahony_Init() {
  OSTaskCreate(TEST_Task_Print_Attitude, NULL, &Stk_PrintMahony[1023],
               TASK_MAHONY_PRINT_PRIO);
  OSTaskCreate(TEST_Mahony, NULL, &Stk_Mahony[1023], TASK_MAHONY_READ_PRIO);
}

#endif