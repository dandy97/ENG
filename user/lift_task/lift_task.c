#include "lift_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart1.h"

#include "rc.h"
#include "can_receive.h"
#include "pid.h"
#include "chassis_task.h"
#include "lift_wheel.h"

// ��������������������0-IDΪ7 �ұ��������1-IDΪ8

//��������״̬
lift_mode_e lift_mode = Init_MODE;
lift_mode_e last_lift_mode = Init_MODE;
pinch_mode_e pinch_mode = PINCH_INIT;
pinch_mode_e last_pinch_mode = PINCH_INIT;
//�ǵ�������ݽṹ
chassis_move_t lift_wheel;
//����������ݽṹ��
lift_move_t lift_move;
//��������ռ�ʣ����
uint32_t lift_high_water;
//static int32_t pidin, pp, pi, pd, sp, si, sd = 0;
void lift_task(void *pvParameters)
{
	//����һ��ʱ��
  vTaskDelay(2000);
	//������ʼ��
	lift_init(&lift_move);
	while(1)
	{
		//�������ݸ���
		lift_feedback_update(&lift_move);
		//��������PID����
		lift_control_loop(&lift_move);
		//���͵���ֵ
		CAN_CMD_LIFT(lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current, lift_move.motor_lift[2].give_current, 0);
		//Ni_Ming(0xf1,lift_move.motor_lift[0].angle ,lift_move.motor_lift[1].angle_set, 0, 0);
		//printf("%d\r\n",HAL_GPIO_ReadPin(EXTEND)<<3 | HAL_GPIO_ReadPin(PINCH)<<2 | HAL_GPIO_ReadPin(FLIP)<<1 | HAL_GPIO_ReadPin(BOUNCE));
		//����Ƶ��4ms
		vTaskDelay(4);
		lift_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//������ʼ��
void lift_init(lift_move_t *lift_init)
{
	if (lift_init == NULL)
	{
		return;
	}
	
	//����ģʽΪ��ʼ��ģʽ
	lift_mode = Init_MODE;
	//�����ٶȻ�PIDֵ
	const static float lift_speed_pid[3] = {400, 0, 180};
	//����λ�û�PIDֵ
	const static float lift_pos_pid[3] = {10, 0, 0};
	
	const static float translation_pos_pid[3] = {9, 0, 0};
	const static float translation_speed_pid[3] = {200, 0, 0};
	
	//��ȡң��ָ�� 
	lift_init->lift_RC = get_remote_control_point();
	
	//��ȡ�����������ָ�� 
	lift_init->motor_lift[0].lift_motor_measure = get_Lift_Motor_Measure_Point(0);
	lift_init->motor_lift[1].lift_motor_measure = get_Lift_Motor_Measure_Point(1);
	lift_init->motor_lift[2].lift_motor_measure = get_Lift_Motor_Measure_Point(2);

	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 10000, 3000);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 1000, 0);
	}
	
	//��ʼ��ƽ��λ�á��ٶȻ�PID
	PID_Init(&lift_init->motor_pos_pid[2], PID_POSITION, translation_pos_pid, 50, 0);
	PID_Init(&lift_init->motor_speed_pid[2], PID_POSITION, translation_speed_pid, 10000, 3000);
	
	//����һ������
  lift_feedback_update(lift_init);
}

static uint32_t init_time = 0;
//�������ݸ���
void lift_feedback_update(lift_move_t *lift_update)
{	
	//��ʼ������
	if(lift_mode == Init_MODE)
	{
		init_time++;
		if(init_time > 1250)
		{
			init_time = 0;
			lift_mode = Ready_MODE;
			lift_update->lift_right_cail  = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->lift_left_cail   = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->translation_cail = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->motor_lift[2].angle = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->translation_cail;
		}
		if(HAL_GPIO_ReadPin(Limit_Switch) == 0)
		{
			init_time = 0;
			lift_mode = Ready_MODE;
			lift_update->lift_right_cail  = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->lift_left_cail   = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->translation_cail = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->motor_lift[2].angle = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->translation_cail;
		}
	}
	//׼������
	if(lift_mode == Ready_MODE || lift_mode == Reset_MODE)
	{
		if(lift_update->motor_lift[2].angle > 29 && lift_update->motor_lift[2].angle < 40)
		{
			lift_mode = Start_MODE;
		}
	}
	//��ʼģʽ
	if(lift_mode != Ready_MODE && lift_mode != Init_MODE && lift_mode != Reset_MODE)
	{
		//������������״̬
		switch(lift_update->lift_RC->rc.s[0])
		{
			case 1:
			{
				if(last_lift_mode == Key_MODE)
				{
					lift_mode = Reset_MODE;//��λģʽ
				}
				else
				{
					lift_mode = Rc_MODE;//ң��ģʽ
				}
				break;
			}			
			case 3:
			{
				if(last_lift_mode == Rc_MODE)
				{
					lift_mode = Reset_MODE;//��λģʽ
				}
				else
				{
					lift_mode = Key_MODE;//����ģʽ
				}
				break;
			}
			case 2:
			{
				lift_mode = Stop_MODE;//ֹͣģʽ
				break;
			}
			default:
			{
				break;
			}
		}
	}
	
	static uint8_t pretect_lift = 0;
	//ȡ��״̬��
	if(lift_update->lift_RC->rc.s[0] == 3)
	{
			if(lift_update->lift_RC->rc.s[1] == 1)
			{
				pretect_lift = 1;
				pinch_mode = PINCH_RISE;
			}
			else if(lift_update->lift_RC->rc.s[1] == 3)
			{
				if(pretect_lift == 1)
				{
					lift_mode = Reset_MODE;//��λģʽ
					pretect_lift = 0;
				}
				pinch_mode = PINCH_INIT;
			}
			else if(lift_update->lift_RC->rc.s[1] == 2)
			{
				pinch_mode = PINCH_GIVE;
			}
	}
	else
	{
			pinch_mode = PINCH_INIT;
	}
	//ң���ź�ʧȥ��Ϊֹͣģʽ
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//ֹͣģʽ
	}
	else if(last_lift_mode == Stop_MODE)
	{
		lift_mode = Init_MODE;
	}
	last_lift_mode = lift_mode;//��һ������״̬
	last_pinch_mode = pinch_mode;//��һ��ȡ��״̬
	//���µ���ٶ�
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[2].speed = lift_update->motor_lift[2].lift_motor_measure->filter_rate / 19.0f;
	
	//���µ���Ƕ�
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->lift_right_cail;//(0~15cm)
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->lift_left_cail;
	lift_update->motor_lift[2].angle = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->translation_cail;
}


//��������PID����
void lift_control_loop(lift_move_t *lift_control)
{	
	lift_control->key_time++;
	/**********************************************���׿���*******************************************************************/
	switch(pinch_mode)
	{
		case PINCH_INIT://��ʼ״̬
		{
			HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EXTEND,GPIO_PIN_RESET);
			break;
		}			
		case PINCH_RISE://ȡ��״̬
		{
			
			Set_LIFT_KEY_GPIO(lift_control->lift_RC->key.v, C, EXTEND, lift_control->key_time);//������������
			contrl_cylinder(lift_control->lift_RC->mouse.press_l, lift_control->lift_RC->mouse.press_r, lift_control->key_time);//������ȡ��
			break;
		}
		case PINCH_GIVE://����״̬
		{
			
			break;
		}
		default:
		{
			break;
		}
	}
	/**********************************************���׿���*******************************************************************/
	/**********************************************�������*******************************************************************/
	switch(lift_mode)
	{
		case Stop_MODE://ֹͣ״̬
		{
			lift_control->motor_lift[2].angle_set = lift_control->motor_lift[2].angle;
			lift_control->motor_lift[2].speed_set = 0;
			break;
		}
		case Init_MODE://��ʼ��ģʽ
		{	
			lift_control->motor_lift[0].angle_set = lift_control->motor_lift[0].angle;
			lift_control->motor_lift[2].speed_set = -20;//-20
			break;
		}
		case Ready_MODE://��ʼģʽ
		{
			lift_control->motor_lift[2].angle_set = -35.5f;
			break;
		}
		case Reset_MODE://��λģʽ
		{
			lift_control->motor_lift[2].angle_set = -35.5f;
			break;
		}
		case Rc_MODE://ң���ָ�״̬
		{
			//����λ������
			lift_control->motor_lift[0].angle_set += lift_control->lift_RC->rc.ch[3] * 0.0005f;		
			lift_control->motor_lift[2].angle_set += lift_control->lift_RC->rc.ch[2] * 0.0005f;//-35
			break;
		}
		case Key_MODE://����ģʽ
		{
				switch(lift_control->lift_RC->rc.s[1])
				{
					case 1://��ʼ״̬
					{
						//����λ������
						lift_control->motor_lift[0].angle_set = 12.0f;
						if(lift_control->motor_lift[0].angle > -10.5f && lift_control->motor_lift[0].angle < -9.0f)
						{
							lift_control->motor_lift[2].angle_set = -60.0f;
						}
						if(lift_control->lift_RC->key.v == Q)//����
						{
							lift_control->motor_lift[2].angle_set -= 0.1f;
						}
						if(lift_control->lift_RC->key.v == E)//����
						{
							lift_control->motor_lift[2].angle_set += 0.1f;
						}
						break;
					}
					case 3://����
					{
						//����λ������
						lift_control->motor_lift[0].angle_set = 1.0f;
						break;
					}
					case 2://����ģʽ
					{
						//����λ������
						lift_control->motor_lift[0].angle_set = 14.0f;	
						break;
					}
					default:
					{
						break; 
					}
				}
			break;
		}
		default:
		{
			break;
		}
	}
	
	//�����߶��޷�(2cm ~ 15cm)
	if(lift_control->motor_lift[0].angle_set > 17.0f)
	{
		lift_control->motor_lift[0].angle_set = 17.0;
	}
	else if(lift_control->motor_lift[0].angle_set < 1.01f)
	{
		lift_control->motor_lift[0].angle_set = 1.0;
	}
	
	//ƽ�ƾ����޷�
	if(lift_control->motor_lift[2].angle_set < -68.1f)
	{
		lift_control->motor_lift[2].angle_set = -68.0;
	}
	else if(lift_control->motor_lift[2].angle_set > -4.99f)
	{
		lift_control->motor_lift[2].angle_set = -5.0;
	}
	
	//�߶�����
	lift_control->motor_lift[1].angle_set = -lift_control->motor_lift[0].angle_set;
	
	//��������PID
	for(uint8_t i = 0; i < 3; i++)
	{
		//λ�û�
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, -lift_control->motor_lift[i].angle_set);
    //�ٶȻ�  
		PID_Calc(&lift_control->motor_speed_pid[i], lift_control->motor_lift[i].speed, lift_control->motor_pos_pid[i].out);
	}
	
	if(lift_mode == Stop_MODE)//ֹͣ״̬
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
		lift_control->motor_lift[2].give_current = PID_Calc(&lift_control->motor_speed_pid[2], lift_control->motor_lift[2].speed, lift_control->motor_lift[2].speed_set); 
	}
	else if(lift_mode == Init_MODE)//��ʼ״̬
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
		lift_control->motor_lift[2].give_current = PID_Calc(&lift_control->motor_speed_pid[2], lift_control->motor_lift[2].speed, lift_control->motor_lift[2].speed_set);
	}
	else if(lift_mode == Ready_MODE)//׼��״̬
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
		lift_control->motor_lift[2].give_current = lift_control->motor_speed_pid[2].out;
	}
	else//����״̬
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
		lift_control->motor_lift[2].give_current = lift_control->motor_speed_pid[2].out;
	}
	/**********************************************�������*******************************************************************/
}

//��������״̬
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}


//ȡ������
void Set_LIFT_KEY_GPIO(uint16_t key, uint16_t key1, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t time)
{
	static uint32_t lift_last_time = 0;
	if((key & key1) && (time - lift_last_time >125))
	{
		lift_last_time = time;
		if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == 0)
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET); 
		}
		else
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET); 
		}
	}
}


//����������
void contrl_cylinder(uint8_t key, uint8_t key1, uint32_t time)
{
	static uint32_t pinch_last_time = 0;
	static int8_t cylinder_state = 0;
	if(key && (time - pinch_last_time >125))//���
	{
		pinch_last_time = time;
		cylinder_state++;
		if(cylinder_state > 5)
		{
			cylinder_state = 0;
		}
	}
	if(key1 && (time - pinch_last_time >125))//�һ�
	{
		pinch_last_time = time;
		cylinder_state--;
		if(cylinder_state < 0)
		{
			cylinder_state = 0;
		}
	}
	if(cylinder_state == 0)//��ʼ
	{
		HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
	}
	if(cylinder_state == 1)//��
	{
		HAL_GPIO_WritePin(FLIP,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
	}
	if(cylinder_state == 2)//��->��
	{
		HAL_GPIO_WritePin(FLIP,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PINCH,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
	}
	if(cylinder_state == 3)//��->��->��
	{
		HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PINCH,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
	}
	if(cylinder_state == 4)//��->��->��->��
	{
		HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
	}
	if(cylinder_state == 5)//��->��->��->��->����
	{
		HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_SET);
	}
}
