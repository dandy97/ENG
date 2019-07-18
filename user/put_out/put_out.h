#ifndef PUT_OUT_H
#define PUT_OUT_H

#include "stm32f4xx_hal.h"

void Put_Out_Init(void);
void Control_Gpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t key);
#endif
