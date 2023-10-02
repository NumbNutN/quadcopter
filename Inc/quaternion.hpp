#include <stdc++.h>

#include <stdio.h>
#include <math.h>
  
using namespace std;

class quaternion{

public:

    double q0;
    double q1;
    double q2;
    double q3;

    quaternion(double q0=1.0f,double q1=0.0f,double q2=0.0f,double q3=0.0f) :q0(q0),q1(q1),q2(q2),q3(q3){
        printf("parameter constructor\n");
    }

    quaternion(const quaternion& quat) {
        printf("copy constructor\n");
    }

    quaternion(quaternion&& quat) {
        printf("move constructor\n");
    }

    quaternion& normalization() {
        double len = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
        q0 = q0 / len;
        q1 = q1 / len;
        q2 = q2 / len;
        q3 = q3 / len;
        return *this;
    }

    friend quaternion operator+(const quaternion& a,const quaternion& b);
    friend quaternion operator*(const quaternion& a,const quaternion& b);

    quaternion& operator=(const quaternion& a) = default;

};

quaternion operator+(const quaternion& a,const quaternion& b) {

    return quaternion(a.q0+b.q0,a.q1+b.q1,a.q2+b.q2+a.q3+b.q3);
    printf("add\n");
}


quaternion operator*(const quaternion& a,const quaternion& b) {
    
    return quaternion(
        a.q0*b.q0 - a.q1*b.q1 - a.q2*b.q2 - a.q3*b.q3,
        a.q1*b.q0 + a.q0*b.q1 - a.q3*b.q2 + a.q2*b.q3,
        a.q2*b.q0 + a.q3*b.q1 + a.q0*b.q2 - a.q1*b.q3,
        a.q3*b.q0 - a.q2*b.q1 + a.q1*b.q2 + a.q0*b.q3
    );
    printf("multiple\n");
}

quaternion operator+(double scale,const quaternion& q) {
    return quaternion(q.q0*scale,q.q1*scale,q.q2*scale,q.q3*scale);
}


