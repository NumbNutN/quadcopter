
#pragma once
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

using namespace std;
class quaternion {
private:
  // std::vector<float> dat;
  float dat[4];

public:
  quaternion(float q0 = 1.0f, float q1 = 0.0f, float q2 = 0.0f, float q3 = 0.0f)
      : dat({q0, q1, q2, q3}) {}

  quaternion(const quaternion &quat) {
    memcpy(dat, quat.dat, sizeof(float) * 4);
  }
  quaternion(quaternion &&quat) { memcpy(dat, quat.dat, sizeof(float) * 4); }
  quaternion normalization() const {
    float len = sqrt(dat[0] * dat[0] + dat[1] * dat[1] + dat[2] * dat[2] +
                     dat[3] * dat[3]);
    return quaternion{dat[0] / len, dat[1] / len, dat[2] / len, dat[3] / len};
  }
  float length() {
    return sqrt(dat[0] * dat[0] + dat[1] * dat[1] + dat[2] * dat[2] +
                dat[3] * dat[3]);
  }
  float getTheta() { return 2 * acos(dat[0]); }
  quaternion getAxis() {
    float sinTheta = sin(acos(dat[0]));
    return quaternion{0, dat[1] / sinTheta, dat[2] / sinTheta,
                      dat[3] / sinTheta};
  }
  quaternion conjugate() const {
    return quaternion{dat[0], -dat[1], -dat[2], -dat[3]};
  }

  quaternion no(size_t idx) const {
      quaternion new_quat = *this;
      new_quat[idx] = 0;
      return new_quat;
  }

  friend quaternion operator+(const quaternion &lhs, const quaternion &rhs);
  friend quaternion operator*(const quaternion &lhs, const quaternion &rhs);
  friend quaternion operator-(const quaternion &lhs, const quaternion &rhs);
  /* 定义叉乘 */
  friend quaternion operator^(const quaternion &lhs, const quaternion &rhs);
  /* 定义点乘 */
  friend float operator&(const quaternion &lhs, const quaternion &rhs);

  template <typename T>
  friend quaternion operator*(T scale, const quaternion &q);
  template <typename T>
  friend quaternion operator/(const quaternion &q, T scale);
  float &operator[](size_t index) { return dat[index]; }
  // read only
  float operator[](size_t index) const { return dat[index]; }

  quaternion &operator=(const quaternion &rhs) = default;
  
  quaternion &operator+=(const quaternion &rhs) {
    dat[0] += rhs[0];
    dat[1] += rhs[1];
    dat[2] += rhs[2];
    dat[3] += rhs[3];
    return *this;
  }
};
inline quaternion operator+(const quaternion &lhs, const quaternion &rhs) {
  return quaternion(lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2], lhs[3] + rhs[3]);
}
inline quaternion operator-(const quaternion &lhs, const quaternion &rhs) {
  return quaternion(lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2], lhs[3] - rhs[3]);
}
inline quaternion operator*(const quaternion &lhs, const quaternion &rhs) {

  return quaternion(lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
                    lhs[1] * rhs[0] + lhs[0] * rhs[1] - lhs[3] * rhs[2] + lhs[2] * rhs[3],
                    lhs[2] * rhs[0] + lhs[3] * rhs[1] + lhs[0] * rhs[2] - lhs[1] * rhs[3],
                    lhs[3] * rhs[0] - lhs[2] * rhs[1] + lhs[1] * rhs[2] + lhs[0] * rhs[3]);
}

// template <typename T>
// concept Scaleable = requires(T x){x*x;}
template <typename T> quaternion operator*(T scale, const quaternion &q) {
  return quaternion(q[0] * scale, q[1] * scale, q[2] * scale, q[3] * scale);
}
template <typename T> quaternion operator/(const quaternion &q, T scale) {
  return quaternion(q[0] / scale, q[1] / scale, q[2] / scale, q[3] / scale);
}
inline ostream &operator<<(ostream &out, const quaternion &q) {
  out << q[1] << " " << q[2] << " " << q[3];
  return out;
}
inline quaternion operator^(const quaternion &lhs, const quaternion &rhs) {
  return quaternion{0, lhs[2] * rhs[3] - lhs[3] * rhs[2], lhs[3] * rhs[1] - lhs[1] * rhs[3],
                    lhs[1] * rhs[2] - lhs[2] * rhs[1]};
}
inline float operator&(const quaternion &lhs, const quaternion &rhs) {
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2] + lhs[3] * rhs[3];
}