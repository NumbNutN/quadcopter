#include "quat_math.hpp"

/* 四阶龙格库塔可以拟合一个函数的近似值 */
quaternion RK4([[maybe_unused]]double t,[[maybe_unused]]const quaternion& q,double delta,const f_t &f)
{
    quaternion k1 = f(t,q);
    quaternion k2 = f(t+0.5*delta,q+0.5*delta*k1);
    quaternion k3 = f(t+0.5*delta,q+0.5*delta*k2);
    quaternion k4 = f(t+delta,q+delta*k3);

    quaternion tmp = 1.0f/6*delta*(k1+2*k2+2*k3+k4);
    quaternion final = q + tmp;
    return final;
}

quaternion RK1([[maybe_unused]]double t,[[maybe_unused]]const quaternion& q,double delta,const f_t &f)
{
    return q + f(t,q) * delta;
}