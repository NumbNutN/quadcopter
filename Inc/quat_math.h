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

#define QUAD_ORIENTATION_MATRIX(q1,q2,q3,q4) { \
    {1,0,0,0}, \
    {0,1-2*q3*q3-2*q4*q4,2*q2*q3-2*q1*q4,2*q1*q3+2*q2*q4}, \
    {0,2*q2*q3+2*q1*q4,1-2*q2*q2-2*q4*q4,2*q3*q4-2*q1*q2}, \
    {0,2*q2*q4-2*q1*q3,2*q1*q2+2*q3*q4,1-2*q2*q2-2*q3*q3} \
    }


extern double cur_quat[4];
extern uint64_t _euler_angle_told_c;

void quat2mat(double* mat,double* quat);
void Runge_Kutta_1st(double* quat,double* gyro,float deltaT);
void quat2eulerAngle_zyx(double* quat,double* x,double* y,double* z);

#endif