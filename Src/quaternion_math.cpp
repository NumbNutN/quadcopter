#include "quaternion.hpp"
#include "myMath.hpp"

#include <math.h>

#include <tuple>

using namespace std;

void print_quaternion2EulerAngle(quaternion& q)
{
    printf("X:%lf\nY:%lf\nZ:%lf\n\n",atan((2*q[0]*q[1] + 2*q[2]*q[3]) / (1-2*q[1]*q[1] - 2*q[2]*q[2])), \
        -asin(2*q[1]*q[3]-2*q[0]*q[2]), \
        atan((2*q[1]*q[2] + 2*q[0]*q[3]) / (1-2*q[2]*q[2] - 2*q[3]*q[3])) );
}

float quat_get_Roll(const quaternion& q){
    float res = -asin(2*q[1]*q[3]-2*q[0]*q[2]);
    while(res > PI || res < -PI){}
    return res;
}

float quat_get_Pitch(const quaternion& q) {
    float res = atan2((2*q[0]*q[1] + 2*q[2]*q[3]),(1-2*q[1]*q[1] - 2*q[2]*q[2]));
    while(res > PI || res < -PI){}
    return res;
}

float quat_get_Yaw(const quaternion& q) {
    float res = atan2((2*q[1]*q[2] + 2*q[0]*q[3]),(1-2*q[2]*q[2] - 2*q[3]*q[3]));
    while(res > PI || res < -PI){}
    return res;
}

quaternion quat2EulerAngle(const quaternion& q) {
    return quaternion{
        0,
        atan((2*q[0]*q[1] + 2*q[2]*q[3]) / (1-2*q[1]*q[1] - 2*q[2]*q[2])),
        -asin(2*q[1]*q[3]-2*q[0]*q[2]),
        atan((2*q[1]*q[2] + 2*q[0]*q[3]) / (1-2*q[2]*q[2] - 2*q[3]*q[3]))
    };
}
