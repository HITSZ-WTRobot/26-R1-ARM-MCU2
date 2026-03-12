#pragma once

#define ARM_MAX_VEL_OUT    50.0f 
#define ARM_MAX_VEL_ROTATE 50.0f 
#define ARM_MAX_VEL_HEIGHT 50.0f 

void Arm_TIM_Callback(void);
void Arm_Init(void);

extern float arm_vel_out;
extern float arm_vel_rotate;
extern float arm_vel_height;
extern float arm_vel_out_last;
extern float arm_vel_rotate_last;
extern float arm_vel_height_last;

void APP_Arm_BeforeUpdate();
void APP_Arm_Update_1kHz();
void APP_Arm_Update_100Hz();
