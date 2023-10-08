#include "quat_math.h"

#include <math.h>

#include "LinearAlgebra/declareFunctions.h"
#include "delay.h"

double cur_quat[4] = {1.0,0,0,0};
uint64_t _euler_angle_told_c;

void _start_count_euler_angle()
{
    _euler_angle_told_c = Get_TimeStamp();
}

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

void Quat_Normalization(double* A,double* C)
{
    double length = sqrt(pow(A[0],2)+pow(A[1],2)+pow(A[2],2)+pow(A[3],2));
    C[0] = A[0] /length;
    C[1] = A[1] /length;
    C[2] = A[2] /length;
    C[3] = A[3] /length;
}

void Runge_Kutta_1st(double* quat,double* gyro,float deltaT)
{
    double q_afterScale[4];
    double q_afterAdd[4];
    double q_afterNorm[4];
    _start_count_euler_angle();
    double runge_kutta_mat[4][3] = RUNGE_KUTTA_1ST_MATRIX(quat[0], quat[1], quat[2], quat[3]);
    mul((double*)runge_kutta_mat,gyro,false,q_afterScale,4,3,1);
    scale(q_afterScale, 0.5*deltaT, 4, 1);
    add(quat, q_afterScale, q_afterAdd, 4, 1, 1);
    Quat_Normalization(q_afterAdd,q_afterNorm);
    memcpy(quat, q_afterNorm,4*sizeof(double));
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

