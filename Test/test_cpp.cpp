#include "quaternion.hpp"


#include <iostream>


using namespace std;

#include "test_RK1.cpp"
#include "test_print_accel.cpp"
#include "test_pid.cpp"
#include "test_pid 3.cpp"
#include "test_motor.cpp"
#include "test_madgrick.cpp"
#include "test_paper_mad.cpp"
#include "test_mahony.cpp"

void TEST_stream()
{
    cout << "hello" << endl;
}

void TEST_quaternion()
{
    quaternion a= {1,0,0,0};
    quaternion b{1,1,1,1};
    quaternion c(1,0,0,0);
    c += b;
}