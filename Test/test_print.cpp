#include "config.h"

#if TEST_PRINT_EN > 0u

#include "os.h"

#include "usart.h"

#include "test_tasks.h"

#include <iostream>

#include "hmc_5583l.hpp"

using namespace std;

OS_STK Stk_Print[1024];

extern hmc_558l* hmc_ptr;
void TEST_Task_Print_Mag(void* arg){
    for(;;)
    {
        quaternion mag = hmc_ptr->getMagnetic();
        cout << "vec " << mag.normalization() << endl;
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

void TEST_Print_Init()
{
    OSTaskCreate(TEST_Task_Print_Mag, (void*)0, &Stk_Print[1023], TASK_PRINT_PRIO);
}

#endif