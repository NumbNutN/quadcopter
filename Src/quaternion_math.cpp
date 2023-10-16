#include "quaternion.hpp"

#include <math.h>

#include <tuple>

using namespace std;


void print_quaternion2EulerAngle(quaternion& q)
{
    printf("X:%lf\nY:%lf\nZ:%lf\n\n",atan((2*q[0]*q[1] + 2*q[2]*q[3]) / (1-2*q[1]*q[1] - 2*q[2]*q[2])), \
        -asin(2*q[1]*q[3]-2*q[0]*q[2]), \
        atan((2*q[1]*q[2] + 2*q[0]*q[3]) / (1-2*q[2]*q[2] - 2*q[3]*q[3])) );
}
