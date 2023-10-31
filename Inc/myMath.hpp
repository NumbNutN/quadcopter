#pragma once

#define PI 3.141592654

#include "quaternion.hpp"

using value_type = quaternion;
using f_t = quaternion(*)(double t,quaternion q);

/* 四阶龙格库塔可以拟合一个函数的近似值 */
quaternion RK4([[maybe_unused]]double t,[[maybe_unused]]const quaternion& q,double delta,const f_t &f);
quaternion RK1([[maybe_unused]]double t,[[maybe_unused]]const quaternion& q,double delta,const f_t &f);

void print_quaternion2EulerAngle(quaternion& q);

float quat_get_Roll(const quaternion& q);

float quat_get_Pitch(const quaternion& q);

float quat_get_Yaw(const quaternion& q);

quaternion quat2EulerAngle(const quaternion& q);

void ano_send_dataFrame(quaternion& attitude);
void ano_send_quaternion(quaternion& attiitude);