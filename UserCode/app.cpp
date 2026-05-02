#include "can.h"
#include "arm.hpp"
#include "cmsis_os2.h"
#include "controller_receive.hpp"
#include "device.hpp"
#include "eventflags.hpp"
#include "tim.h"

static osTimerId_t controller_timHandle = nullptr;

extern "C" void Controller_softTIM(void* argument)
{
    (void)argument;
    TIM10_Callback(nullptr);
}

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    (void)htim;

    Arm_TIM_Callback();
    APP_Device_Update();
}

extern "C" void Init(void* argument)
{
    (void)argument;
    flags_create();
    APP_Device_Init();
    Arm_Init();
    Controller_receiver_Init();

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_Base_Start_IT(&htim6);

    controller_timHandle = osTimerNew(Controller_softTIM, osTimerPeriodic, NULL, NULL);
    osTimerStart(controller_timHandle, 1);

    osThreadExit();
}
