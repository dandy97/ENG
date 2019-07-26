#include "tof.h"

UART_HandleTypeDef huart6;
static tof_data_t tof_data;
static uint8_t Tof_1_data[1][128];
//TX
void Tof_Init(void)
{
  huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;	
  HAL_UART_Init(&huart6);
	
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)Tof_1_data, 128);	
}

static uint32_t xiuer_r = 0;
void USART6_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart6.Instance->DR;
      tmp = huart6.Instance->SR;
      tmp--;
      CLEAR_BIT(huart6.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart6.hdmarx);
			
     	uint16_t  temp = huart6.hdmarx->Instance->NDTR;  
			if((128 - temp) == 9)//传输数组长度 - 传输的剩余数据项数
			{
				if(Tof_1_data[0][0] == 0x59)
				{
					if(Tof_1_data[0][1] == 0x59)
					{
						xiuer_r = Tof_1_data[0][5]<<8 | Tof_1_data[0][4];
						if((xiuer_r > 100) && (xiuer_r != 65535))
						tof_data.tof_h = Tof_1_data[0][3]<<8 | Tof_1_data[0][2];
						//printf("%d\r\n",tof_data.dis_r);
					}
				}
			}
			HAL_UART_Receive_DMA(&huart6, (uint8_t *)Tof_1_data, 128);
			SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
			DMA1->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;//DMA中断清0
			__HAL_DMA_SET_COUNTER(huart6.hdmarx, 128); //重载NDTR位
			__HAL_DMA_ENABLE(huart6.hdmarx);
		}
		else if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE))
		{
      uint16_t tmp = huart6.Instance->SR;
      tmp = huart6.Instance->DR;
      tmp--;
		}
}

//返回tof变量地址，通过指针方式获取原始数据
const tof_data_t *get_tof_Info_Measure_Point(void)
{
	return &tof_data;
}
