#pragma once

#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include "arm.hpp"
#include "clamp.hpp"
#include "eventflags.hpp"

#define MAX_JOYSTICK                   2000.0f // 遥控器摇杆数据最大值
#define __JOYSTICK2VEL_ROLL__(__JOYSTICK__)        (__JOYSTICK__ * MAX_VEL_ROLL / MAX_JOYSTICK)
#define __JOYSTICK2VEL_YAW__(__JOYSTICK__)         (__JOYSTICK__ * MAX_VEL_YAW / MAX_JOYSTICK)
#define __JOYSTICK2VEL_ARM_PUSHOUT__(__JOYSTICK__) (__JOYSTICK__ * ARM_MAX_VEL_OUT / MAX_JOYSTICK)
#define __JOYSTICK2VEL_ARM_ROTATE__(__JOYSTICK__)  (__JOYSTICK__ * ARM_MAX_VEL_ROTATE / MAX_JOYSTICK)
#define __JOYSTICK2VEL_ARM_HEIGHT__(__JOYSTICK__)  (__JOYSTICK__ * ARM_MAX_VEL_HEIGHT / MAX_JOYSTICK)

uint8_t CRC8(uint8_t* buffer, uint8_t len);
// void    Decode(void* argument);
void Buffer_Decode(void);
void controller_task(void* argument);

typedef enum
{
    CHASSIS_MODE = 0,
    CLAMP_MODE   = 1,
    ARM_MODE     = 2,
    AUTO_ALIGN_MODE = 3
} JOYSTICK_MODE_E;

void Controller_receiver_Init(void);
void TIM10_Callback(TIM_HandleTypeDef* htim);
