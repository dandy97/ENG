#include "put_out.h"
#include "stm32f4xx_hal.h"

void Put_Out_Init(void)
{
	/*88-PD7  89-PB3  95-PB8  96-PB9 */
	/*41-PE10 42-PE11 43-PE12 44-PE13*/
	/*1 -PE2  2 -PE3  3 -PE4  98-PE1 */
	/*93-PB7  97-PE0                 */
	/*17-PC2  18-PC3  25-PA2  26-PA3 */
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	//上层继电器 PE1、PE2、PE3、PE4
	GPIO_Initure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4; 
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; 
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4,GPIO_PIN_RESET);
	
	//下层继电器 PB3、PB8、PB9
	GPIO_Initure.Pin = GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9; 
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; 
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9,GPIO_PIN_RESET);
	
	//下层继电器 PD7
	GPIO_Initure.Pin = GPIO_PIN_7; 
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; 
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
	
	//上层磁性开关 PC2、PC3
	GPIO_Initure.Pin = GPIO_PIN_2 | GPIO_PIN_3; 
	GPIO_Initure.Mode = GPIO_MODE_INPUT;  
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
	//上层磁性开关 PA2、PA3
	GPIO_Initure.Pin = GPIO_PIN_2 | GPIO_PIN_3; 
	GPIO_Initure.Mode = GPIO_MODE_INPUT;  
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	//上层限位开关
	GPIO_Initure.Pin = GPIO_PIN_0; 
	GPIO_Initure.Mode = GPIO_MODE_INPUT;  
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
	//下层磁性开关 PE10、PE11、PE12、PE13
	GPIO_Initure.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13; 
	GPIO_Initure.Mode = GPIO_MODE_INPUT;  
	GPIO_Initure.Pull = GPIO_PULLDOWN;          
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}
