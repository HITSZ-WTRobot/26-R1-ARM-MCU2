#pragma once

#include "dji.hpp"

extern motors::DJIMotor* motor_clamp_out;
extern motors::DJIMotor* motor_clamp_roll_left;
extern motors::DJIMotor* motor_clamp_roll_right;
extern motors::DJIMotor* motor_clamp_catch;
extern motors::DJIMotor* motor_clamp_yaw;
extern motors::DJIMotor* rotate_motor;
extern motors::DJIMotor* catch_motor;
extern motors::DJIMotor* raiseandlower_motor;

void APP_Device_Init(void);
void APP_Device_Update(void);
bool APP_Device_isAllConnected();
void APP_Device_WaitConnections();
