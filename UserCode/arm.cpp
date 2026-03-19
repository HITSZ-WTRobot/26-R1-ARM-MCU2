#include "arm.hpp"

#include "device.hpp"
#include "eventflags.hpp"
#include "interboard_comm.hpp"
#include "main.h"
#include "clamp.hpp"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "pump_ctrl.hpp"
#include "tim.h"

#include <cstdint>

#define ARM_RESET_ANGLE 0.0f
#define ARM_CATCH_PUSH_ANGLE 250.0f
#define ARM_CATCH_PUSH_ANGLE_MAX 280.0f
#define ARM_CATCH_HEIGHT_LOW -300.0f
#define ARM_CATCH_HEIGHT_MID 400.0f
#define ARM_CATCH_HEIGHT_HIGH 1050.0f
#define ARM_RELEASE_HEIGHT 800.0f
#define ARM_ROTATE_ANGLE 320.0f

#define ARM_AUTO_WAIT_HEIGHT_MS 600U
#define ARM_AUTO_WAIT_ROTATE_MS 300U
#define ARM_AUTO_WAIT_PUMP_ON_MS 100U
#define ARM_AUTO_WAIT_PUSH_MS 500U
#define ARM_AUTO_WAIT_RELEASE_HEIGHT_MS 600U
#define ARM_AUTO_WAIT_ROTATE_BACK_MS 900U
#define ARM_AUTO_WAIT_RELEASE_MS 250U
#define ARM_AUTO_RETRACT_PUSH_ANGLE 0.0f
#define ARM_AUTO_RETREAT_SPEED_MPS 0.25f
#define ARM_AUTO_RETREAT_MIN_TIME_MS 300U
#define ARM_AUTO_RETREAT_MAX_TIME_MS 2500U

#define PUMP_VALVE_GPIO_Port GPIOE
#define PUMP_VALVE_Pin GPIO_PIN_3
#define PUMP_RELAY_GPIO_Port GPIOE
#define PUMP_RELAY_Pin GPIO_PIN_4

using Motor_PosCtrl_t = controllers::MotorPosController;
using Motor_VelCtrl_t = controllers::MotorVelController;

Pump_Config_t pump_config = {
    .htim = &htim3,
    .channel = TIM_CHANNEL_3,
    .valve_port = PUMP_VALVE_GPIO_Port,
    .pump_port = PUMP_RELAY_GPIO_Port,
    .valve_pin = PUMP_VALVE_Pin,
    .pump_pin = PUMP_RELAY_Pin,
    .invert = 1,
};

static Pump_t pump;

enum AutoCatchState {
  AUTO_CATCH_IDLE = 0,
  AUTO_CATCH_GO_HEIGHT,
  AUTO_CATCH_ROTATE_AND_PUMP,
  AUTO_CATCH_PUSH_OUT,
  AUTO_CATCH_VEHICLE_RETREAT,
  AUTO_CATCH_GO_RELEASE_HEIGHT,
  AUTO_CATCH_ROTATE_BACK,
  AUTO_CATCH_RELEASE,
};

static volatile AutoCatchState g_auto_catch_state = AUTO_CATCH_IDLE;
static volatile uint32_t g_auto_catch_state_start_ms = 0;
static volatile float g_auto_catch_target_height = ARM_CATCH_HEIGHT_LOW;
static volatile float g_auto_retreat_length_m = 0.15f;

__attribute__((weak)) void Arm_AutoVehicleMove(float retreat_length_m,
                                               bool enable) {
  (void)retreat_length_m;
  InterboardComm_SendRetreatCommand(enable);
}

osTimerId_t arm_timHandle = nullptr;
float arm_pos_height = ARM_CATCH_HEIGHT_LOW;

float arm_vel_out = 0;
float arm_vel_out_last = 0;
float arm_vel_rotate = 0;
float arm_vel_rotate_last = 0;
float arm_vel_height = 0;
float arm_vel_height_last = 0;

osThreadId_t ArmHandle = nullptr;
const osThreadAttr_t arm_attributes = {
    .name = "arm",
    .stack_size = 128 * 8,
    .priority = (osPriority_t)osPriorityNormal1,
};

Motor_PosCtrl_t *pos_rotate_motor = nullptr;
Motor_PosCtrl_t *pos_raiseandlower_motor = nullptr;
Motor_PosCtrl_t *pos_catch_motor = nullptr;

Motor_VelCtrl_t *vel_rotate_motor = nullptr;
Motor_VelCtrl_t *vel_raiseandlower_motor = nullptr;
Motor_VelCtrl_t *vel_catch_motor = nullptr;

void Arm_TIM_Callback(void) {
  pos_raiseandlower_motor->update();
  pos_catch_motor->update();
  pos_rotate_motor->update();
  vel_raiseandlower_motor->update();
  vel_rotate_motor->update();
  vel_catch_motor->update();
}

static void Arm_Contrl_Task(void *argument) {
  (void)argument;
  for (;;) {
    if (arm_vel_out_last != 0) {
      pos_catch_motor->disable();
      vel_catch_motor->enable();
      vel_catch_motor->setRef(arm_vel_out);
    }
    if (arm_vel_height_last != 0) {
      pos_raiseandlower_motor->disable();
      vel_raiseandlower_motor->enable();
      vel_raiseandlower_motor->setRef(arm_vel_height);
    }
    if (arm_vel_rotate_last != 0) {
      pos_rotate_motor->disable();
      vel_rotate_motor->enable();
      vel_rotate_motor->setRef(arm_vel_rotate);
    }
    osDelay(10);
  }
}

static uint32_t AutoRetreatDurationMs(float retreat_length_m) {
  if (retreat_length_m < 0.0f) {
    retreat_length_m = -retreat_length_m;
  }
  float ms = retreat_length_m / ARM_AUTO_RETREAT_SPEED_MPS * 1000.0f;
  if (ms < (float)ARM_AUTO_RETREAT_MIN_TIME_MS) {
    ms = (float)ARM_AUTO_RETREAT_MIN_TIME_MS;
  }
  if (ms > (float)ARM_AUTO_RETREAT_MAX_TIME_MS) {
    ms = (float)ARM_AUTO_RETREAT_MAX_TIME_MS;
  }
  return (uint32_t)ms;
}

static bool AutoStepTimeout(uint32_t wait_ms, uint32_t now_ms) {
  return (uint32_t)(now_ms - g_auto_catch_state_start_ms) >= wait_ms;
}

static uint32_t AutoRotateAndPumpWaitMs() {
  return (ARM_AUTO_WAIT_ROTATE_MS > ARM_AUTO_WAIT_PUMP_ON_MS)
             ? ARM_AUTO_WAIT_ROTATE_MS
             : ARM_AUTO_WAIT_PUMP_ON_MS;
}

static void AutoCatchEnterState(AutoCatchState state, uint32_t now_ms) {
  g_auto_catch_state = state;
  g_auto_catch_state_start_ms = now_ms;
}

bool Arm_AutoCatchStart(ArmAutoCatchLevel level) {
  if (g_auto_catch_state != AUTO_CATCH_IDLE) {
    return false;
  }

  switch (level) {
  case ARM_AUTO_CATCH_LOW:
    g_auto_catch_target_height = ARM_CATCH_HEIGHT_LOW;
    break;
  case ARM_AUTO_CATCH_MID:
    g_auto_catch_target_height = ARM_CATCH_HEIGHT_MID;
    break;
  case ARM_AUTO_CATCH_HIGH:
    g_auto_catch_target_height = ARM_CATCH_HEIGHT_HIGH;
    break;
  default:
    return false;
  }

  arm_vel_out = 0;
  arm_vel_rotate = 0;
  arm_vel_height = 0;
  arm_vel_out_last = 0;
  arm_vel_rotate_last = 0;
  arm_vel_height_last = 0;
  Arm_AutoVehicleMove(g_auto_retreat_length_m, false);

  AutoCatchEnterState(AUTO_CATCH_GO_HEIGHT, HAL_GetTick());
  return true;
}

bool Arm_AutoCatchBusy() { return g_auto_catch_state != AUTO_CATCH_IDLE; }

void Arm_AutoCatchAbortKeepPump() {
  if (g_auto_catch_state == AUTO_CATCH_IDLE) {
    return;
  }

  Arm_AutoVehicleMove(g_auto_retreat_length_m, false);

  if (pos_catch_motor) {
    pos_catch_motor->disable();
  }
  if (vel_catch_motor) {
    vel_catch_motor->disable();
  }
  if (pos_rotate_motor) {
    pos_rotate_motor->disable();
  }
  if (vel_rotate_motor) {
    vel_rotate_motor->disable();
  }
  if (pos_raiseandlower_motor) {
    pos_raiseandlower_motor->disable();
  }
  if (vel_raiseandlower_motor) {
    vel_raiseandlower_motor->disable();
  }

  arm_vel_out = 0;
  arm_vel_rotate = 0;
  arm_vel_height = 0;
  arm_vel_out_last = 0;
  arm_vel_rotate_last = 0;
  arm_vel_height_last = 0;
  g_auto_catch_state = AUTO_CATCH_IDLE;
}

void Arm_SetAutoRetreatLength(float length_m) {
  if (length_m < 0.05f) {
    length_m = 0.05f;
  }
  g_auto_retreat_length_m = length_m;
}

static void Arm_softTIM(void *argument) {
  (void)argument;

  static uint8_t pump_state = 0;
  static uint8_t height_state = 0;
  static uint8_t rotate_state = 0;

  const uint32_t now_ms = HAL_GetTick();

  if (g_auto_catch_state != AUTO_CATCH_IDLE) {
    switch (g_auto_catch_state) {
    case AUTO_CATCH_GO_HEIGHT:// 开始自动抓取流程，先去目标高度
      vel_raiseandlower_motor->disable();
      pos_raiseandlower_motor->enable();
      pos_raiseandlower_motor->setRef(g_auto_catch_target_height);
      if (AutoStepTimeout(ARM_AUTO_WAIT_HEIGHT_MS, now_ms)) {
        AutoCatchEnterState(AUTO_CATCH_ROTATE_AND_PUMP, now_ms);
      }
      break;

    case AUTO_CATCH_ROTATE_AND_PUMP: // 到达目标高度后并行旋转和吸泵
      vel_rotate_motor->disable();
      pos_rotate_motor->enable();
      pos_rotate_motor->setRef(ARM_ROTATE_ANGLE);
      Pump_Catch(&pump, 1);
      if (AutoStepTimeout(AutoRotateAndPumpWaitMs(), now_ms)) {
        AutoCatchEnterState(AUTO_CATCH_PUSH_OUT, now_ms);
      }
      break;

    case AUTO_CATCH_PUSH_OUT: // 吸稳后继续向前推一段距离，增加抓取稳定性
      vel_catch_motor->disable();
      pos_catch_motor->enable();
      pos_catch_motor->setRef(ARM_CATCH_PUSH_ANGLE);
      if (AutoStepTimeout(ARM_AUTO_WAIT_PUSH_MS, now_ms)) {
        Arm_AutoVehicleMove(g_auto_retreat_length_m, true);
        AutoCatchEnterState(AUTO_CATCH_VEHICLE_RETREAT, now_ms);
      }
      break;

    case AUTO_CATCH_VEHICLE_RETREAT: // 推出后退一段距离，避免物体被抓起后贴着墙壁等障碍物
      Arm_AutoVehicleMove(g_auto_retreat_length_m, true);
      if (AutoStepTimeout(AutoRetreatDurationMs(g_auto_retreat_length_m), now_ms)) {
        Arm_AutoVehicleMove(g_auto_retreat_length_m, false);
        AutoCatchEnterState(AUTO_CATCH_GO_RELEASE_HEIGHT, now_ms);
      }
      break;

    case AUTO_CATCH_GO_RELEASE_HEIGHT: // 退完后去释放高度准备放下物体
      vel_raiseandlower_motor->disable();
      pos_raiseandlower_motor->enable();
      pos_raiseandlower_motor->setRef(ARM_RELEASE_HEIGHT);
      if (AutoStepTimeout(ARM_AUTO_WAIT_RELEASE_HEIGHT_MS, now_ms)) {
        AutoCatchEnterState(AUTO_CATCH_ROTATE_BACK, now_ms);
      }
      break;

    case AUTO_CATCH_ROTATE_BACK: // 到达释放高度后旋转回初始位置准备放下物体
      vel_rotate_motor->disable();
      pos_rotate_motor->enable();
      pos_rotate_motor->setRef(ARM_RESET_ANGLE);
      if (AutoStepTimeout(ARM_AUTO_WAIT_ROTATE_BACK_MS, now_ms)) {
        AutoCatchEnterState(AUTO_CATCH_RELEASE, now_ms);
      }
      break;

    case AUTO_CATCH_RELEASE: // 旋转回初始位置后关闭吸泵放下物体
      vel_catch_motor->disable();
      pos_catch_motor->enable();
      pos_catch_motor->setRef(ARM_AUTO_RETRACT_PUSH_ANGLE);
      Pump_Release(&pump, 1);
      if (AutoStepTimeout(ARM_AUTO_WAIT_RELEASE_MS, now_ms)) {
        Arm_AutoVehicleMove(g_auto_retreat_length_m, false);
        arm_vel_out = 0;
        arm_vel_rotate = 0;
        arm_vel_height = 0;
        arm_vel_out_last = 0;
        arm_vel_rotate_last = 0;
        arm_vel_height_last = 0;
        g_auto_catch_state = AUTO_CATCH_IDLE;
      }
      break;

    case AUTO_CATCH_IDLE:
    default:
      break;
    }
    return;
  }

  if ((osEventFlagsWait(flags_id, 0x00000008U, osFlagsWaitAny, 0) &
       0xFF000008U) == 0x00000008U) {
    vel_raiseandlower_motor->disable();
    pos_raiseandlower_motor->enable();
    switch (height_state) {
    case 0:
      arm_pos_height = ARM_CATCH_HEIGHT_LOW;
      break;
    case 1:
      arm_pos_height = ARM_CATCH_HEIGHT_MID;
      break;
    default:
      arm_pos_height = ARM_CATCH_HEIGHT_HIGH;
      break;
    }
    pos_raiseandlower_motor->setRef(arm_pos_height);
    height_state = (height_state + 1) % 3;
  }

  if ((osEventFlagsWait(flags_id, 0x00000020U, osFlagsWaitAny, 0) &
       0xFF000020U) == 0x00000020U) {
    vel_rotate_motor->disable();
    pos_rotate_motor->enable();
    if (rotate_state == 0) {
      pos_rotate_motor->setRef(ARM_RESET_ANGLE);
      rotate_state = 1;
    } else {
      pos_rotate_motor->setRef(ARM_ROTATE_ANGLE);
      rotate_state = 0;
    }
  }

  if ((osEventFlagsWait(flags_id, 0x00000040U, osFlagsWaitAny, 0) &
       0xFF000040U) == 0x00000040U) {
    if (pump_state == 0) {
      pump_state = 1;
      Pump_Catch(&pump, 1);
    } else {
      pump_state = 0;
      Pump_Release(&pump, 1);
    }
  }
}

void Arm_Init(void) {
  (void)ARM_CATCH_PUSH_ANGLE;
  (void)ARM_CATCH_PUSH_ANGLE_MAX;
  (void)ARM_RELEASE_HEIGHT;

  Pump_Init(&pump, &pump_config);

  controllers::MotorVelController::Config arm_catch_vel_cfg{};
  arm_catch_vel_cfg.pid.Kp = 25.0f;
  arm_catch_vel_cfg.pid.Ki = 0.15f;
  arm_catch_vel_cfg.pid.Kd = 20.0f;
  arm_catch_vel_cfg.pid.abs_output_max = 5000.0f;

  controllers::MotorVelController::Config arm_rotate_vel_cfg{};
  arm_rotate_vel_cfg.pid.Kp = 100.0f;
  arm_rotate_vel_cfg.pid.Ki = 0.8f;
  arm_rotate_vel_cfg.pid.Kd = 20.0f;
  arm_rotate_vel_cfg.pid.abs_output_max = 8000.0f;

  controllers::MotorVelController::Config arm_raiseandlower_vel_cfg{};
  arm_raiseandlower_vel_cfg.pid.Kp = 100.0f;
  arm_raiseandlower_vel_cfg.pid.Ki = 0.8f;
  arm_raiseandlower_vel_cfg.pid.Kd = 1.0f;
  arm_raiseandlower_vel_cfg.pid.abs_output_max = 8000.0f;

  controllers::MotorPosController::Config arm_catch_pos_cfg{};
  arm_catch_pos_cfg.velocity_pid.Kp = 100.0f;
  arm_catch_pos_cfg.velocity_pid.Ki = 0.5f;
  arm_catch_pos_cfg.velocity_pid.Kd = 0.9f;
  arm_catch_pos_cfg.velocity_pid.abs_output_max = 5000.0f;
  arm_catch_pos_cfg.position_pid.Kp = 1.0f;
  arm_catch_pos_cfg.position_pid.Ki = 0.002f;
  arm_catch_pos_cfg.position_pid.Kd = 0.8f;
  arm_catch_pos_cfg.position_pid.abs_output_max = 500.0f;
  arm_catch_pos_cfg.pos_vel_freq_ratio = 1;

  controllers::MotorPosController::Config arm_rotate_pos_cfg{};
  arm_rotate_pos_cfg.velocity_pid.Kp = 100.0f;
  arm_rotate_pos_cfg.velocity_pid.Ki = 0.5f;
  arm_rotate_pos_cfg.velocity_pid.Kd = 0.9f;
  arm_rotate_pos_cfg.velocity_pid.abs_output_max = 8000.0f;
  arm_rotate_pos_cfg.position_pid.Kp = 24.5f;
  arm_rotate_pos_cfg.position_pid.Ki = 0.42f;
  arm_rotate_pos_cfg.position_pid.Kd = 100.0f;
  arm_rotate_pos_cfg.position_pid.abs_output_max = 500.0f;
  arm_rotate_pos_cfg.pos_vel_freq_ratio = 10;

  controllers::MotorPosController::Config arm_raiseandlower_pos_cfg{};
  arm_raiseandlower_pos_cfg.velocity_pid.Kp = 100.0f;
  arm_raiseandlower_pos_cfg.velocity_pid.Ki = 0.001f;
  arm_raiseandlower_pos_cfg.velocity_pid.Kd = 0.5f;
  arm_raiseandlower_pos_cfg.velocity_pid.abs_output_max = 8000.0f;
  arm_raiseandlower_pos_cfg.position_pid.Kp = 2.0f;
  arm_raiseandlower_pos_cfg.position_pid.Ki = 0.01f;
  arm_raiseandlower_pos_cfg.position_pid.Kd = 0.20f;
  arm_raiseandlower_pos_cfg.position_pid.abs_output_max = 2000.0f;
  arm_raiseandlower_pos_cfg.pos_vel_freq_ratio = 1;

  vel_catch_motor = new Motor_VelCtrl_t(catch_motor, arm_catch_vel_cfg);
  pos_catch_motor = new Motor_PosCtrl_t(catch_motor, arm_catch_pos_cfg);
  vel_rotate_motor = new Motor_VelCtrl_t(rotate_motor, arm_rotate_vel_cfg);
  pos_rotate_motor = new Motor_PosCtrl_t(rotate_motor, arm_rotate_pos_cfg);
  vel_raiseandlower_motor =
      new Motor_VelCtrl_t(raiseandlower_motor, arm_raiseandlower_vel_cfg);
  pos_raiseandlower_motor =
      new Motor_PosCtrl_t(raiseandlower_motor, arm_raiseandlower_pos_cfg);

  pos_catch_motor->disable();
  vel_catch_motor->disable();
  pos_raiseandlower_motor->disable();
  vel_raiseandlower_motor->disable();
  vel_rotate_motor->disable();
  pos_rotate_motor->disable();

  ArmHandle = osThreadNew(Arm_Contrl_Task, NULL, &arm_attributes);
  arm_timHandle = osTimerNew(Arm_softTIM, osTimerPeriodic, NULL, NULL);
  osTimerStart(arm_timHandle, 10);
}

void APP_Arm_BeforeUpdate() { Arm_Init(); }

void APP_Arm_Update_1kHz() { Arm_TIM_Callback(); }

void APP_Arm_Update_100Hz() {}
