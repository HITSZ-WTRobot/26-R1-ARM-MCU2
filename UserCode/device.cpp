#include "device.hpp"

#include "can.h"
#include "cmsis_os2.h"
#include "dji.hpp"

motors::DJIMotor* motor_clamp_out        = nullptr;
motors::DJIMotor* motor_clamp_yaw        = nullptr;
motors::DJIMotor* motor_clamp_roll_left  = nullptr;
motors::DJIMotor* motor_clamp_roll_right = nullptr;
motors::DJIMotor* motor_clamp_catch      = nullptr;
motors::DJIMotor* rotate_motor           = nullptr;
motors::DJIMotor* raiseandlower_motor    = nullptr;
motors::DJIMotor* catch_motor            = nullptr;

namespace
{

void can_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    motors::DJIMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::DJIMotor::CANBaseReceiveCallback);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config motor_clamp_out_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 1,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_roll_left_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 2,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_roll_right_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 3,
    .auto_zero = true,
    .reverse   = true,
};

constexpr motors::DJIMotor::Config motor_clamp_yaw_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M3508_C620,
    .id1       = 4,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config motor_clamp_catch_config = {
    .hcan      = &hcan2,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 5,
    .auto_zero = true,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config rotate_motor_config = {
    .hcan      = &hcan1,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 1,
    .auto_zero = false,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config catch_motor_config = {
    .hcan      = &hcan1,
    .type      = motors::DJIMotor::Type::M2006_C610,
    .id1       = 2,
    .auto_zero = false,
    .reverse   = false,
};

constexpr motors::DJIMotor::Config raiseandlower_motor_config = {
    .hcan      = &hcan1,
    .type      = motors::DJIMotor::Type::M3508_C620,
    .id1       = 3,
    .auto_zero = false,
    .reverse   = false,
};

void motor_init()
{
    using motors::DJIMotor;

    motor_clamp_out        = new DJIMotor(motor_clamp_out_config);
    motor_clamp_roll_left  = new DJIMotor(motor_clamp_roll_left_config);
    motor_clamp_roll_right = new DJIMotor(motor_clamp_roll_right_config);
    motor_clamp_catch      = new DJIMotor(motor_clamp_catch_config);
    motor_clamp_yaw        = new DJIMotor(motor_clamp_yaw_config);
    rotate_motor           = new DJIMotor(rotate_motor_config);
    catch_motor            = new DJIMotor(catch_motor_config);
    raiseandlower_motor    = new DJIMotor(raiseandlower_motor_config);
}

} // namespace

void APP_Device_Init()
{
    can_init();
    motor_init();
}

void APP_Device_Update()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    motors::DJIMotor::SendIqCommand(&hcan2, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
}

bool APP_Device_isAllConnected()
{
    bool all_connected = true;

    all_connected &= (motor_clamp_out != nullptr) && motor_clamp_out->isConnected();
    all_connected &= (motor_clamp_roll_left != nullptr) && motor_clamp_roll_left->isConnected();
    all_connected &= (motor_clamp_roll_right != nullptr) && motor_clamp_roll_right->isConnected();
    all_connected &= (motor_clamp_catch != nullptr) && motor_clamp_catch->isConnected();
    all_connected &= (motor_clamp_yaw != nullptr) && motor_clamp_yaw->isConnected();
    all_connected &= (rotate_motor != nullptr) && rotate_motor->isConnected();
    all_connected &= (catch_motor != nullptr) && catch_motor->isConnected();
    all_connected &= (raiseandlower_motor != nullptr) && raiseandlower_motor->isConnected();

    return all_connected;
}

void APP_Device_WaitConnections()
{
    while (!APP_Device_isAllConnected())
        osDelay(1);
}
