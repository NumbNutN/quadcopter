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
    std::vector<double> vec;

public:

    quaternion(double q0=1.0f,double q1=0.0f,double q2=0.0f,double q3=0.0f) :dat({q0,q1,q2,q3}){
        //cout <<"parameter constructor"<< endl;
    }

    // quaternion(std::initializer_list<double>& list) :dat(list){
    //     //cout <<"initializer list constructor" << endl;
    // }

    quaternion(const quaternion& quat) {
        memcpy(dat, quat.dat, sizeof(double)*4);
        //dat = quat.dat;
        //printf("copy constructor\n");
    }

    quaternion(quaternion&& quat) {
        memcpy(dat, quat.dat, sizeof(double)*4);
        //dat = std::vector<double>(std::move(quat.dat));
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

    friend quaternion operator+(const quaternion& a,const quaternion& b);
    friend quaternion operator*(const quaternion& a,const quaternion& b);
    friend quaternion operator*(double scale,const quaternion& q);
    friend quaternion operator*(int scale, const quaternion &q);
    friend quaternion operator*(long unsigned int scale, const quaternion &q);

    double operator[](size_t index) const{
        return dat[index];
    }

    quaternion& operator=(const quaternion& a) = default;

    quaternion& operator+=(const quaternion& a){
        dat[0] += a.dat[0];dat[1] += a.dat[1];dat[2] += a.dat[2];dat[3]+=a.dat[3];
        return *this;
    }
};

inline quaternion operator+(const quaternion& a,const quaternion& b) {

    return quaternion(a.dat[0]+b.dat[0],a.dat[1]+b.dat[1],a.dat[2]+b.dat[2]+a.dat[3]+b.dat[3]);
    //printf("add\n");
}


inline quaternion operator*(const quaternion& a,const quaternion& b) {
    
    return quaternion(
        a.dat[0]*b.dat[0] - a.dat[1]*b.dat[1] - a.dat[2]*b.dat[2] - a.dat[3]*b.dat[3],
        a.dat[1]*b.dat[0] + a.dat[0]*b.dat[1] - a.dat[3]*b.dat[2] + a.dat[2]*b.dat[3],
        a.dat[2]*b.dat[0] + a.dat[3]*b.dat[1] + a.dat[0]*b.dat[2] - a.dat[1]*b.dat[3],
        a.dat[3]*b.dat[0] - a.dat[2]*b.dat[1] + a.dat[1]*b.dat[2] + a.dat[0]*b.dat[3]
    );
    //printf("multiple\n");
}

inline quaternion operator*(double scale,const quaternion& q) {
    return quaternion(q.dat[0]*scale,q.dat[1]*scale,q.dat[2]*scale,q.dat[3]*scale);
}

inline quaternion operator*(int scale, const quaternion &q){
    return quaternion(q.dat[0]*scale,q.dat[1]*scale,q.dat[2]*scale,q.dat[3]*scale);
}

inline quaternion operator*(long unsigned int scale, const quaternion &q){
    return quaternion(q.dat[0]*scale,q.dat[1]*scale,q.dat[2]*scale,q.dat[3]*scale);
}

inline ostream& operator<<(ostream& out,const quaternion& q) {
    out << q[0] << " " << q[1] << " " << q[2] << " " << q[3];
    return out;
}