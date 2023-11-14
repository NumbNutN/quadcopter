#include "config.h"

#if TEST_PRINT_EN > 0u

#include "os.h"

#include "usart.h"

#include "test_tasks.h"

#include <iostream>
#include <fstream>

#include "hmc_5583l.hpp"
#include "mpu6050.hpp"

using namespace std;

OS_STK Stk_Print[1024];

extern hmc_558l* hmc_ptr;
extern mpu6050* mpu6050_ptr;

void TEST_Task_Print_Accel_Mag(void* arg){
    
    ofstream usart;
    usart.open("usart");
    for(;;)
    {
        quaternion mag = hmc_ptr->getMagnetic().normalization();
        usart <<  mag << " " << mag[1]+mag[2]+mag[3] << endl;
        // quaternion accel = mpu6050_ptr->get_acceleration();
        // cout << "a " << accel.normalization() << endl << endl << endl;
        OSTimeDlyHMSM(0, 0, 0, 200);
    }
}

#include <iomanip>
void TEST_Print_Init()
{
    //cout<<setiosflags(ios::fixed)<<setprecision(2);
    OSTaskCreate(TEST_Task_Print_Accel_Mag, (void*)0, &Stk_Print[1023], TASK_PRINT_PRIO);
}

#endif