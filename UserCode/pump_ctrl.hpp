#pragma once

#include "gpio.h"
#include "tim.h"

#include <cstdint>

typedef struct
{
    TIM_HandleTypeDef* htim;       
    uint32_t           channel;    
    GPIO_TypeDef*      valve_port; 
    GPIO_TypeDef*      pump_port;  
    uint16_t           valve_pin;  
    uint16_t           pump_pin;   
    uint8_t            invert;     
} Pump_Config_t;

typedef struct
{
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
    GPIO_TypeDef*      valve_port;
    GPIO_TypeDef*      pump_port;
    uint16_t           valve_pin;
    uint16_t           pump_pin;
    uint8_t            invert;
} Pump_t;

void Pump_Init(Pump_t* hpump, const Pump_Config_t* config);
void Pump_SetPower(Pump_t* hpump, uint8_t power);

void Pump_ValveOn(Pump_t* hpump);
void Pump_ValveOff(Pump_t* hpump);

void Pump_RelayOn(Pump_t* hpump);
void Pump_RelayOff(Pump_t* hpump);

void Pump_Catch(Pump_t* hpump, uint8_t enable);
void Pump_Release(Pump_t* hpump, uint8_t enable);
