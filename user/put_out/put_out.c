#include "put_out.h"
#include "stm32f4xx_hal.h"

void Put_Out_Init(void)
{
	/*95-PB8 96-PB9 88-PD7 89-PB3*/
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	//磁性开关 PB3、PB8、PB9、PD7
	GPIO_Initure.Pin = GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9; 
	GPIO_Initure.Mode = GPIO_MODE_INPUT; 
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	//磁性开关 PB3、PB8、PB9、PD7
	GPIO_Initure.Pin = GPIO_PIN_7; 
	GPIO_Initure.Mode = GPIO_MODE_INPUT; 
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	
	//继电器IO口 PE10、PE11、PE12、PE13
	GPIO_Initure.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13; 
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13,GPIO_PIN_RESET); 
}
