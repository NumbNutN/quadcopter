#include "quaternion.hpp"

using value_type = quaternion;
using f_t = quaternion(*)(quaternion omega,quaternion q);

extern quaternion last_gyro;
extern quaternion cur_gyro;
extern quaternion cur_quat;
extern uint32_t delta;

quaternion quaternion_derivative(quaternion omega,quaternion q)
{
    return 0.5*omega*q;
}

/* 四阶龙格库塔可以拟合一个函数的近似值 */
quaternion RK4(uint32_t t,const quaternion& q,const f_t &f)
{
    quaternion omega_averageVale = 0.5*(last_gyro + cur_gyro);
    quaternion k1 = 0.5*f(last_gyro,cur_quat);
    quaternion k2 = 0.5*f(omega_averageVale,cur_quat + 0.5*delta*k1);
    quaternion k3 = 0.5*f(omega_averageVale,cur_quat + 0.5*delta*k2);
    quaternion k4 = 0.5*f(cur_gyro,cur_quat + delta*k3);

    return cur_quat + 1.0f/6*delta*(k1+2*k2+2*k3+k4);
}