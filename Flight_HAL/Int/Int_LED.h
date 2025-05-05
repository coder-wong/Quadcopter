#ifndef __Int_LED
#define __Int_LED

#include "main.h"

void Int_LED_On(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Int_LED_Off(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Int_LED_Toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
