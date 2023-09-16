#include "quat_math.h"

#include <math.h>

#include "LinearAlgebra/declareFunctions.h"

void quat2mat(double* mat,double* quat)
{
    *(mat + 4*0 + 0) = *(mat + 4*1 + 1) = *(mat + 4*2 + 2) = *(mat + 4*3 + 3) = quat[0];
    *(mat + 4*1 + 0) = *(mat + 4*3 + 2) = quat[1];
    *(mat + 4*1 + 3) = *(mat + 4*2 + 0) = quat[2];
    *(mat + 4*2 + 1) = *(mat + 4*3 + 0) = quat[3];
    *(mat + 4*0 + 1) = *(mat + 4*2 + 3) = -quat[1];
    *(mat + 4*0 + 2) = *(mat + 4*3 + 1) = -quat[2];
    *(mat + 4*0 + 3) = *(mat + 4*1 + 2) = -quat[3];
}

void Runge_Kutta_1st(double* newquat,double* quat,double* gyro,float deltaT)
{
    double runge_kutta_mat[4][3] = RUNGE_KUTTA_1ST_MATRIX(quat[0], quat[1], quat[2], quat[3]);
    mul((double*)runge_kutta_mat,gyro,false,newquat,4,3,1);
    scale(newquat, 0.5*deltaT, 4, 1);
    add(newquat, newquat, quat, 4, 1, 1);
}

void quat2eulerAngle_zyx(double* quat,double* x,double* y,double* z)
{
    double a = quat[0];
    double b = quat[1];
    double c = quat[2];
    double d = quat[3];
    *y = -asin(2*b*d-2*a*c);
    *x = atan((2*a*b + 2*c*d) / (1-2*b*b - 2*c*c));
    *z = atan((2*b*c + 2*a*d) / (1-2*c*c - 1-2*d*d));
}