#include "clamp.hpp"

#include "main.h"

#include <cmath>

#define CAMERA2MACHINE_X 0
#define CAMERA2MACHINE_Y 0
#define CAMERA2MACHINE_YAW 0

#define AprilTag2MACHINE_X 0
#define AprilTag2MACHINE_Y 0

Motor_PosCtrl_t *clamp_out_pos = nullptr;
Motor_PosCtrl_t *clamp_yaw_pos = nullptr;
Motor_PosCtrl_t *clamp_rollleft_pos = nullptr;
Motor_PosCtrl_t *clamp_rollright_pos = nullptr;
Motor_PosCtrl_t *clamp_catch_pos = nullptr;

Motor_VelCtrl_t *clamp_out_vel = nullptr;
Motor_VelCtrl_t *clamp_yaw_vel = nullptr;
Motor_VelCtrl_t *clamp_rollleft_vel = nullptr;
Motor_VelCtrl_t *clamp_rollright_vel = nullptr;
Motor_VelCtrl_t *clamp_catch_vel = nullptr;

enum Control_Mode clamp_out_mode;
enum Control_Mode clamp_roll_mode;

static uint32_t time_now;
static uint32_t time_lockedstart;
bool control_reset = false;

float target_out = 0;
float target_yaw = 0;
float target_roll = 0;

float clamp_vel_out = 0;
float clamp_vel_yaw = 0;
float clamp_vel_roll = 0;

float catch_angle = 0;

osTimerId_t drawer_timHandle;

enum PROCESS reset_status = error;

osThreadId_t clampHandle = nullptr;
const osThreadAttr_t clamp_attributes = {
    .name = "clamp",
    .stack_size = 128 * 8,
    .priority = (osPriority_t)osPriorityNormal1,
};

void Clamp_Init(void) {
  controllers::MotorVelController::Config clamp_out_vel_cfg{};
  clamp_out_vel_cfg.pid.Kp = 30.0f;
  clamp_out_vel_cfg.pid.Ki = 0.1f;
  clamp_out_vel_cfg.pid.Kd = 15.0f;
  clamp_out_vel_cfg.pid.abs_output_max = 3000.0f;

  controllers::MotorVelController::Config clamp_yaw_vel_cfg{};
  clamp_yaw_vel_cfg.pid.Kp = 100.0f;
  clamp_yaw_vel_cfg.pid.Ki = 0.8f;
  clamp_yaw_vel_cfg.pid.Kd = 1.0f;
  clamp_yaw_vel_cfg.pid.abs_output_max = 8000.0f;

  controllers::MotorVelController::Config clamp_roll_vel_cfg{};
  clamp_roll_vel_cfg.pid.Kp = 25.0f;
  clamp_roll_vel_cfg.pid.Ki = 0.15f;
  clamp_roll_vel_cfg.pid.Kd = 20.0f;
  clamp_roll_vel_cfg.pid.abs_output_max = 5000.0f;

  controllers::MotorVelController::Config clamp_catch_vel_cfg{};
  clamp_catch_vel_cfg.pid.Kp = 20.0f;
  clamp_catch_vel_cfg.pid.Ki = 1.5f;
  clamp_catch_vel_cfg.pid.Kd = 0.15f;
  clamp_catch_vel_cfg.pid.abs_output_max = 5000.0f;

  controllers::MotorPosController::Config clamp_out_pos_cfg{};
  clamp_out_pos_cfg.velocity_pid.Kp = 30.0f;
  clamp_out_pos_cfg.velocity_pid.Ki = 0.10f;
  clamp_out_pos_cfg.velocity_pid.Kd = 15.00f;
  clamp_out_pos_cfg.velocity_pid.abs_output_max = 4000.0f;
  clamp_out_pos_cfg.position_pid.Kp = 2.0f;
  clamp_out_pos_cfg.position_pid.Ki = 0.0085f;
  clamp_out_pos_cfg.position_pid.Kd = 2.5f;
  clamp_out_pos_cfg.position_pid.abs_output_max = 100.0f;
  clamp_out_pos_cfg.pos_vel_freq_ratio = 10;

  controllers::MotorPosController::Config clamp_roll_pos_cfg{};
  clamp_roll_pos_cfg.velocity_pid.Kp = 30.0f;
  clamp_roll_pos_cfg.velocity_pid.Ki = 0.10f;
  clamp_roll_pos_cfg.velocity_pid.Kd = 15.0f;
  clamp_roll_pos_cfg.velocity_pid.abs_output_max = 5000.0f;
  clamp_roll_pos_cfg.position_pid.Kp = 2.0f;
  clamp_roll_pos_cfg.position_pid.Ki = 0.0085f;
  clamp_roll_pos_cfg.position_pid.Kd = 2.50f;
  clamp_roll_pos_cfg.position_pid.abs_output_max = 30.0f;
  clamp_roll_pos_cfg.pos_vel_freq_ratio = 10;

  controllers::MotorPosController::Config clamp_yaw_pos_cfg{};
  clamp_yaw_pos_cfg.velocity_pid.Kp = 12.0f;
  clamp_yaw_pos_cfg.velocity_pid.Ki = 0.20f;
  clamp_yaw_pos_cfg.velocity_pid.Kd = 5.00f;
  clamp_yaw_pos_cfg.velocity_pid.abs_output_max = 8000.0f;
  clamp_yaw_pos_cfg.position_pid.Kp = 80.0f;
  clamp_yaw_pos_cfg.position_pid.Ki = 1.00f;
  clamp_yaw_pos_cfg.position_pid.Kd = 0.00f;
  clamp_yaw_pos_cfg.position_pid.abs_output_max = 2000.0f;
  clamp_yaw_pos_cfg.pos_vel_freq_ratio = 10;

  controllers::MotorPosController::Config clamp_catch_pos_cfg{};
  clamp_catch_pos_cfg.velocity_pid.Kp = 30.0f;
  clamp_catch_pos_cfg.velocity_pid.Ki = 0.10f;
  clamp_catch_pos_cfg.velocity_pid.Kd = 15.0f;
  clamp_catch_pos_cfg.velocity_pid.abs_output_max = 2000.0f;
  clamp_catch_pos_cfg.position_pid.Kp = 2.0f;
  clamp_catch_pos_cfg.position_pid.Ki = 0.0085f;
  clamp_catch_pos_cfg.position_pid.Kd = 2.5f;
  clamp_catch_pos_cfg.position_pid.abs_output_max = 500.0f;
  clamp_catch_pos_cfg.pos_vel_freq_ratio = 10;

  clamp_out_vel = new Motor_VelCtrl_t(motor_clamp_out, clamp_out_vel_cfg);
  clamp_yaw_vel = new Motor_VelCtrl_t(motor_clamp_yaw, clamp_yaw_vel_cfg);
  clamp_rollleft_vel =
      new Motor_VelCtrl_t(motor_clamp_roll_left, clamp_roll_vel_cfg);
  clamp_rollright_vel =
      new Motor_VelCtrl_t(motor_clamp_roll_right, clamp_roll_vel_cfg);
  clamp_catch_vel = new Motor_VelCtrl_t(motor_clamp_catch, clamp_catch_vel_cfg);
  clamp_out_pos = new Motor_PosCtrl_t(motor_clamp_out, clamp_out_pos_cfg);
  clamp_rollleft_pos =
      new Motor_PosCtrl_t(motor_clamp_roll_left, clamp_roll_pos_cfg);
  clamp_rollright_pos =
      new Motor_PosCtrl_t(motor_clamp_roll_right, clamp_roll_pos_cfg);
  clamp_yaw_pos = new Motor_PosCtrl_t(motor_clamp_yaw, clamp_yaw_pos_cfg);
  clamp_catch_pos = new Motor_PosCtrl_t(motor_clamp_catch, clamp_catch_pos_cfg);

  clamp_out_pos->disable();
  clamp_rollleft_pos->disable();
  clamp_rollright_pos->disable();
  clamp_catch_pos->enable();
  clamp_yaw_pos->disable();
  clamp_out_vel->disable();
  clamp_rollleft_vel->disable();
  clamp_rollright_vel->disable();
  clamp_yaw_vel->enable();

  clamp_out_mode = VEL_Control;
  clamp_roll_mode = VEL_Control;
}

void Clamp_Control_Init(void) {
  clampHandle = osThreadNew(Clamp_Control, NULL, &clamp_attributes);
  drawer_timHandle = osTimerNew(Clamp_softTIM, osTimerPeriodic, NULL, NULL);
  osTimerStart(drawer_timHandle, 10); // 10 ms periodic timer
}

void Clamp_TIM_Callback(void) {
  switch (clamp_out_mode) {
  case POS_Control:
    clamp_out_vel->disable();
    clamp_out_pos->enable();
    clamp_out_pos->setRef(target_out);
    clamp_out_pos->update();
    break;

  case VEL_Control:
    clamp_out_pos->disable();
    clamp_out_vel->enable();
    clamp_out_vel->setRef(clamp_vel_out);
    clamp_out_vel->update();
    break;

  default:
    break;
  }

  switch (clamp_roll_mode) {
  case POS_Control:
    clamp_rollleft_vel->disable();
    clamp_rollleft_pos->enable();
    clamp_rollright_vel->disable();
    clamp_rollright_pos->enable();
    clamp_rollleft_pos->setRef(target_roll);
    clamp_rollright_pos->setRef(target_roll);
    clamp_rollleft_pos->update();
    clamp_rollright_pos->update();
    break;

  case VEL_Control:
    clamp_rollright_pos->disable();
    clamp_rollright_vel->enable();
    clamp_rollleft_pos->disable();
    clamp_rollleft_vel->enable();
    clamp_rollright_vel->setRef(clamp_vel_roll);
    clamp_rollleft_vel->setRef(clamp_vel_roll);
    clamp_rollleft_vel->update();
    clamp_rollright_vel->update();
    break;

  default:
    break;
  }

  clamp_catch_pos->setRef(catch_angle);
  clamp_catch_pos->update();
  clamp_yaw_vel->setRef(clamp_vel_yaw);
  clamp_yaw_vel->update();
}

static void Reset(void) {
  clamp_out_mode = VEL_Control;
  clamp_vel_out = 100;
  while (reset_status != success) {
    reset_status = wait;
    if (std::fabs(clamp_out_vel->getPID().getOutput()) >= 1500.0f) {
      reset_status = processing;
      time_lockedstart = HAL_GetTick();
    }
    while (reset_status == processing) {
      if (std::fabs(clamp_out_vel->getPID().getOutput()) >= 1500.0f) {
        time_now = HAL_GetTick();
      } else {
        reset_status = wait;
        break;
      }
      if (time_now - time_lockedstart >= 500) {
        reset_status = success;
        break;
      }
    }
    if (reset_status == success) {
      break;
    }
    osDelay(1);
  }
}

void Clamp_Control(void *argument) {
  for (;;) {
    if (control_reset == 1) {
      Reset();
      clamp_vel_out = 0;
      if (motor_clamp_out != nullptr) {
        motor_clamp_out->resetAngle();
      }
      clamp_out_vel->getPID().reset();
      control_reset = false;
    }
    osDelay(100);
  }
}

/**
 *
 * @param arguement
 */
void Clamp_softTIM(void *arguement) {
  (void)arguement;

  if ((osEventFlagsWait(flags_id, 0x00000004U, osFlagsWaitAny, 0) &
       0xFF000004U) == 0x00000004U) {
    static int toggle = 0;
    if (toggle == 0) {
      catch_angle = 200;
      toggle = 1;
    } else {
      catch_angle = 0;
      toggle = 0;
      clamp_catch_vel->getPID().reset();
    }
  }

  if ((osEventFlagsWait(flags_id, 0x00000080U, osFlagsWaitAny, 0) &
       0xFF000080U) == 0x00000080U) {
    static int toggle = 0;
    if (toggle == 0) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
      toggle = 1;
    } else {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
      toggle = 0;
    }
  }
}

void APP_Clamp_BeforeUpdate() {
  Clamp_Init();
  Clamp_Control_Init();
}

void APP_Clamp_Update_1kHz() { Clamp_TIM_Callback(); }

void APP_Clamp_Update_100Hz() {}
