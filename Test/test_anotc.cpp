#include "config.h"

#ifdef TEST_ANOTC_EN

#include "os.h"
#include <stdint.h>

#include "test_tasks.h"

#include "quat_math.hpp"
#include "anotc.hpp"
#include "quaternion.hpp"
#include "mpu6050.hpp"

OS_STK Stk_PrintMahony[1024];

extern quaternion attitude;
extern mpu6050* mpu6050_ptr;

/* 欧拉角 */
#define ANO_ATTITUDE_EULERANGLE_FMT 0x03
#define ANO_ATTITUDE_ENLERANGLE_DATLEN 0x07
void send_euler_angle(){
  auto frame =
    anotcDataFrame<ANO_ATTITUDE_ENLERANGLE_DATLEN>(ANO_ATTITUDE_EULERANGLE_FMT);
  int8_t data[7];
    int16_t roll = quat_get_Roll(attitude) * 100 * 57.3;
    int16_t pitch = quat_get_Pitch(attitude) * 100 * 57.3;
    int16_t yaw = quat_get_Yaw(attitude) * 100 * 57.3;
    *(int16_t*)data = roll;
    *(int16_t*)(data+2) = pitch;
    *(int16_t*)(data+4) = yaw;
    *(data+6)=0x01;
    frame.pack(data);
    frame.send();
}

/* 惯性传感器数据 */
#define ANO_SENSOR_FMT 0x01
#define ANO_SENSOR_DATLEN 0x0D
void mpu6050_send_sensor(){
    auto accel = mpu6050_ptr->_accelerometer_data_ptr;
    auto gyro = mpu6050_ptr->_gyroscope_data_ptr;
    int8_t sensor_data[13];
    auto sensor_frame = 
    anotcDataFrame<ANO_SENSOR_DATLEN>(ANO_SENSOR_FMT);

    *(((int16_t*)sensor_data)) = ((accel[0] << 8) | accel[1])/16384;
    *(((int16_t*)sensor_data)+1) = ((accel[2] << 8) | accel[3])/16384;
    *(((int16_t*)sensor_data)+2) = ((accel[4] << 8) | accel[5])/16384;
    *(((int16_t*)sensor_data)+3) = ((int16_t)((gyro[0] << 8) | gyro[1]))/65.5;
    *(((int16_t*)sensor_data)+4) = ((int16_t)((gyro[2] << 8) | gyro[3]))/65.5;
    *(((int16_t*)sensor_data)+5) = ((int16_t)((gyro[4] << 8) | gyro[5]))/65.5;

    sensor_frame.pack(sensor_data);
    sensor_frame.send();
}

/* 角速度 */
#define ANO_GYRO 0xF2
#define ANO_GYRO_DATLEN 0x06
void send_gyro(){
    int8_t data[ANO_GYRO_DATLEN];
    auto gyro = mpu6050_ptr->_gyroscope_data_ptr;
    auto frame =
    anotcDataFrame<ANO_GYRO_DATLEN>(ANO_GYRO);
    
    *(int16_t*)data = ((int16_t)((gyro[0] << 8) | gyro[1]))/0.65500f;
    *(int16_t*)(data+2) = ((int16_t)((gyro[2] << 8) | gyro[3]))/0.65500f;
    *(int16_t*)(data+4) = ((int16_t)((gyro[4] << 8) | gyro[5]))/0.65500f;

    frame.pack(data);
    frame.send();
}

/* 外环输出值 */
#define ANO_EXTERNAL_OUTPUT 0xF1
#define ANO_EXTERNAL_DATLEN 0x06
#include "pid.hpp"
extern pid ExternalControllerList[3];
void send_ext_output(){
    int8_t data[ANO_EXTERNAL_DATLEN];
    auto frame =
    anotcDataFrame<ANO_EXTERNAL_DATLEN>(ANO_EXTERNAL_OUTPUT);
    
    *(int16_t*)data = ExternalControllerList[1].getOutput()*5732.4;
    *(int16_t*)(data+2) = ExternalControllerList[0].getOutput()*5732.4;
    *(int16_t*)(data+4) = 0;

    frame.pack(data);
    frame.send();
}


/* 采集当前内环输出值和角加速度的关联 */
#ifdef PID3_SERIE
#define ANO_ROLL_INT_ANGULAR_ACCEL 0xF3
#define ANO_ROLL_INT_ANGULAR_ACCEL_DATLEN 0x04
extern pid InternalControllerList[3];
void inter_out_and_angular_acceleration(){
    auto frame =
        anotcDataFrame<ANO_ROLL_INT_ANGULAR_ACCEL_DATLEN>(ANO_ROLL_INT_ANGULAR_ACCEL);
    /* 内环输出值 */
    int8_t dat[ANO_ROLL_INT_ANGULAR_ACCEL_DATLEN];
    *(((int16_t*)dat)) = InternalControllerList[0].getOutput()*10000;

    /* 角加速度 */
    float cur_gyroX = mpu6050_ptr->get_current_gyro()[1];
    float last_gyroX = mpu6050_ptr->get_last_gyro()[1];
    float angular_accleration = (cur_gyroX - last_gyroX)/mpu6050_ptr->getSamplePeriod();
    *(((int16_t*)dat)+1) = angular_accleration*100;

    frame.pack(dat);
    frame.send();
}
#endif
void TEST_Task_Info_Tran(void *arg) {

  for (;;) {
    send_euler_angle();
    send_gyro();
    send_ext_output(); 
    //inter_out_and_angular_acceleration();
    OSTimeDlyHMSM(0, 0, 0, 50);
  }
}

void TEST_Anotc_Conn_Init() {
  OS_ERR err;
  OSTaskCreate(TEST_Task_Info_Tran, NULL, &Stk_PrintMahony[511],
               TASK_ANOTC_INFO_TRAN_PRIO);
  OSTaskNameSet(TASK_ANOTC_INFO_TRAN_PRIO, (INT8U *)"PRINT_ATTITUDE", &err);
}

#endif