#ifndef _MATH_H
#define _MATH_H

#include <stdint.h>

#define ROTATION_MATRIX_ZYX(x,y,z)  { \
    {cos(z)*cos(y),-cos(x)*sin(z)+sin(y)*sin(x)*cos(z),sin(x)*sin(z)+sin(y)*sin(x)*cos(z)}, \
    {cos(y)*sin(z),cos(x)*cos(z)+sin(y)*sin(x)*sin(z),-sin(x)*cos(z)+sin(y)*cos(x)*sin(z)}, \
    {-sin(y),cos(y)*sin(x),cos(y)*cos(x)} \
    }

#define RUNGE_KUTTA_1ST_MATRIX(q1,q2,q3,q4) { \
    {-q2,-q3,-q4}, \
    {q1,-q4,q3}, \
    {q4,q1,-q2}, \
    {-q3,q2,q1} \
    }


extern double cur_quat[4];
extern uint64_t _euler_angle_told;

void quat2mat(double* mat,double* quat);
void Runge_Kutta_1st(double* quat,double* gyro,float deltaT);
void quat2eulerAngle_zyx(double* quat,double* x,double* y,double* z);

#endif