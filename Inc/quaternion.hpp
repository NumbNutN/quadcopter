#pragma once

#include <iostream>
#include <vector>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
  
using namespace std;

class quaternion{

private:
    
    //std::vector<double> dat;
    double dat[4];

public:

    quaternion(double q0=1.0f,double q1=0.0f,double q2=0.0f,double q3=0.0f) :dat({q0,q1,q2,q3}){
        //cout <<"parameter constructor"<< endl;
    }

    // quaternion(std::initializer_list<double>& list) :dat(list){
    //     //cout <<"initializer list constructor" << endl;
    // }

    quaternion(const quaternion& quat) {
        memcpy(dat, quat.dat, sizeof(double)*4);
        //dat = quat;
        //printf("copy constructor\n");
    }

    quaternion(quaternion&& quat) {
        memcpy(dat, quat.dat, sizeof(double)*4);
        //dat = std::vector<double>(std::move(quat));
        //printf("move constructor\n");
    }

    quaternion& normalization() {
        double len = sqrt(dat[0]*dat[0]+dat[1]*dat[1]+dat[2]*dat[2]+dat[3]*dat[3]);
        dat[0] = dat[0] / len;
        dat[1] = dat[1] / len;
        dat[2] = dat[2] / len;
        dat[3] = dat[3] / len;
        return *this;
    }

    double length() {
        return sqrt(dat[0]*dat[0]+dat[1]*dat[1]+dat[2]*dat[2]+dat[3]*dat[3]);
    }

    friend quaternion operator+(const quaternion& a,const quaternion& b);
    friend quaternion operator*(const quaternion& a,const quaternion& b);
    friend quaternion operator-(const quaternion& a,const quaternion& b);
    
    template <typename T>
    friend quaternion operator*(T scale, const quaternion &q);

    template <typename T>
    friend quaternion operator/(const quaternion &q,T scale);

    double operator[](size_t index) const{
        return dat[index];
    }

    quaternion& operator=(const quaternion& a) = default;

    quaternion& operator+=(const quaternion& a){
        dat[0] += a[0];dat[1] += a[1];dat[2] += a[2];dat[3]+=a[3];
        return *this;
    }
};

inline quaternion operator+(const quaternion& a,const quaternion& b) {

    return quaternion(a[0]+b[0],a[1]+b[1],a[2]+b[2],a[3]+b[3]);
}

inline quaternion operator-(const quaternion& a,const quaternion& b) {

    return quaternion(a[0]-b[0],a[1]-b[1],a[2]-b[2],a[3]-b[3]);
}

inline quaternion operator*(const quaternion& a,const quaternion& b) {
    
    return quaternion(
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[1]*b[0] + a[0]*b[1] - a[3]*b[2] + a[2]*b[3],
        a[2]*b[0] + a[3]*b[1] + a[0]*b[2] - a[1]*b[3],
        a[3]*b[0] - a[2]*b[1] + a[1]*b[2] + a[0]*b[3]
    );
}

template <typename T>
quaternion operator*(T scale, const quaternion &q) {
    return quaternion(q[0]*scale,q[1]*scale,q[2]*scale,q[3]*scale);
}

template <typename T>
quaternion operator/(const quaternion &q,T scale) {
    return quaternion(q[0]/scale,q[1]/scale,q[2]/scale,q[3]/scale);
}

inline ostream& operator<<(ostream& out,const quaternion& q) {
    out << q[0] << " " << q[1] << " " << q[2] << " " << q[3];
    return out;
}