#include "pump_ctrl.hpp"

static void Pump_SetPowerInternal(Pump_t* hpump, uint8_t power)
{
    if (hpump == nullptr || hpump->htim == nullptr)
    {
        return;
    }

    if (power > 100)
    {
        power = 100;
    }

    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(hpump->htim) + 1;
    uint32_t       pulse  = period * power / 100;

    if (hpump->invert)
    {
        pulse = period - pulse;
    }

    __HAL_TIM_SET_COMPARE(hpump->htim, hpump->channel, pulse);
}

void Pump_Init(Pump_t* hpump, const Pump_Config_t* config)
{
    if (hpump == nullptr || config == nullptr)
    {
        return;
    }

    hpump->htim       = config->htim;
    hpump->channel    = config->channel;
    hpump->valve_port = config->valve_port;
    hpump->pump_port  = config->pump_port;
    hpump->pump_pin   = config->pump_pin;
    hpump->valve_pin  = config->valve_pin;
    hpump->invert     = config->invert;
}

void Pump_SetPower(Pump_t* hpump, uint8_t power)
{
    Pump_SetPowerInternal(hpump, power);
}

void Pump_ValveOn(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->valve_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_SET);
}

void Pump_ValveOff(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->valve_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->valve_port, hpump->valve_pin, GPIO_PIN_RESET);
}

void Pump_RelayOn(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->pump_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_SET);
}

void Pump_RelayOff(Pump_t* hpump)
{
    if (hpump == nullptr || hpump->pump_port == nullptr)
    {
        return;
    }
    HAL_GPIO_WritePin(hpump->pump_port, hpump->pump_pin, GPIO_PIN_RESET);
}

void Pump_Catch(Pump_t* hpump, uint8_t enable)
{
    if (!enable)
    {
        return;
    }
    Pump_ValveOff(hpump);
    Pump_RelayOn(hpump);
}

void Pump_Release(Pump_t* hpump, uint8_t enable)
{
    if (!enable)
    {
        return;
    }
    Pump_ValveOn(hpump);
    Pump_RelayOff(hpump);
}
