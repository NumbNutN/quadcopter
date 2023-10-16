#pragma once

#include "quaternion.hpp"

template <typename T>
T gradient_decent(const T& x,T(*grad_func)(const T& x) ,float step) {
    T gradient = grad_func(x);
    return x - step / gradient.length() * gradient;
}
