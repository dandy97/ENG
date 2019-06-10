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
lift_mode_t lift_mode = Init_MODE;
//�ǵ�������ݽṹ
chassis_move_t lift_wheel;
//����������ݽṹ��
lift_move_t lift_move;

//��������ռ�ʣ����
uint32_t lift_high_water;
void lift_task(void *pvParameters)
{
	//����һ��ʱ��
  vTaskDelay(2000);
	//������ʼ��
	lift_init(&lift_move);
	//�ǵ������ʼ��
	lift_wheel_init(&lift_wheel);
	while(1)
	{
		//�������ݸ���
		lift_feedback_update(&lift_move);
		//����ģʽѡ��
		lift_mode_switch(&lift_move);
		//��������PID����
		lift_control_loop(&lift_move);
		
		lift_wheel_control_loop(&lift_wheel);
		//���͵���ֵ
	  CAN_CMD_LIFT(lift_wheel.motor_chassis[0].give_current, lift_wheel.motor_chassis[1].give_current, lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current);
		//CAN_CMD_LIFT(lift_wheel.motor_chassis[0].give_current, lift_wheel.motor_chassis[1].give_current, 0, 0);
		//Ni_Ming(0xf1,lift_move.motor_lift[1].give_current,0,0,0);
		//CAN_CMD_LIFT(0, 0, -0, 0);
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
	//�����ٶȻ�PIDֵ
	const static float lift_speed_pid[3] = {20, 1, 600};
	//����λ�û�PIDֵ
	const static float lift_pos_pid[3] = {8, 0, 0};
	
	//��ȡң��ָ�� 
	lift_init->lift_RC = get_remote_control_point();
	
	//��ȡ�����������ָ�� 
	lift_init->motor_lift[0].lift_motor_measure = get_Motor_Measure_Point(6);
	lift_init->motor_lift[1].lift_motor_measure = get_Motor_Measure_Point(7);

	//������ʼ״̬Ϊ��ֹ״̬
	lift_init->mode = Lift_Stop;
	
	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 7000, 3000);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 500, 0);
	}
	
	//����һ������
  lift_feedback_update(lift_init);
}

//�������ݸ���
void lift_feedback_update(lift_move_t *lift_update)
{
	//���µ���ٶ�
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	
	//���µ���Ƕ�
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle;
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle;
	
	//������������״̬
	switch(lift_update->lift_RC->rc.s[0])
	{
		case 1:
		{
			lift_mode = Rc_MODE;
			break;
		}			
		case 3:
		{
			lift_mode = Key_MODE;
			break;
		}
		case 2:
		{
			lift_mode = Stop_MODE;
			break;
		}
		default:
		{
			break;
		}
	}
	
}

//����ģʽѡ��
void lift_mode_switch(lift_move_t *lift_mode)
{
	//��ǰϵͳʱ�����������ʱ��
	static uint32_t lift_task_system_time,life_task_time = 0;
	
	//ÿ4ms�ۼ�һ
	life_task_time++;
	
	//��ȡ��ǰϵͳʱ��
	lift_task_system_time = xTaskGetTickCount();
	//�������ϵͳ��ǰʱ���ȥ����ң���жϵ�ǰʱ�䣬˵��û���յ�ң���ź�
	if((lift_task_system_time - lift_mode->lift_RC->time) > 88)
	{
		//ֹͣ״̬
		lift_mode->mode = Lift_Stop;
	}
	
	//��߲��˴������棬����״̬����ֹͣ״̬����������ʱ�����500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[1] == 1) && (life_task_time > 125))
	{
		//����״̬
		lift_mode->mode = Lift_Key;
	}
	
	//��߲��˴������£�����״̬����ֹͣ״̬����������ʱ�����500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[1] == 3) && (life_task_time > 125))
	{
		//����״̬
		lift_mode->mode = Lift_Run;
	}
	
	//��߲��˴������£�����״̬����ֹͣ״̬����������ʱ�����500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[1] == 2) && (life_task_time > 125))
	{
		//����״̬
		lift_mode->mode = Lift_Give;
	}
	
	//�ұ߲��˴��м䣬����״̬����ֹͣ״̬����������ʱ�����500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[0] == 2) && (life_task_time > 125))
	{
		//ң���ָ�״̬
		lift_mode->mode = Lift_Rc;       
	}

	else
	{
		//ֹͣ״̬
		lift_mode->mode = Lift_Stop;
	}
}


//ȡ��״̬
static uint8_t pinch_state = 0;
//����ģʽʱ�� ��һ�ΰ��°���ʱ�� 
static uint32_t lift_keyboard_time, lift_keyboard_last_time, lift_last_Press_time, release_delay, auto_pinch_time= 0;
//ȡ�������߶�
static uint32_t pinch_height = 765;//780
//�Զ�ȡ����־
static uint8_t auto_pinch = 0;
//��������PID����
void lift_control_loop(lift_move_t *lift_control)
{	
	static uint32_t lift_time_delay = 0;
	switch(lift_control->mode)
	{
		case Lift_Stop://ֹͣ״̬
		{
			lift_time_delay = 0;
			//����λ������
			lift_control->motor_lift[1].angle_set = 100;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			break;
		}
		case Lift_Rc://ң���ָ�״̬
		{
			lift_time_delay = 0;
			//����λ������
			lift_control->motor_lift[1].angle_set += lift_control->lift_RC->rc.ch[3] * 0.002f;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;		
			break;
		}
		case Lift_Run://ƽʱģʽ
		{
			//1���� PC5
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
			//�ɿ���ȡ
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			//������ҩ��
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			//2����
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
			
			//����λ������
			lift_control->motor_lift[1].angle_set = 120;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;		
			break;
		}
		case Lift_Give://����ģʽ
		{
			lift_control->motor_lift[1].angle_set = 950;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			
			static uint8_t okok = 0;
			static uint32_t okok_delay = 0;
			if(lift_control->motor_lift[1].angle_set > 300)
			{
				//Q ȡ������ PB0
				if(lift_control->lift_RC->key.v & Q)
				{
					//����
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				}
				//һ������
				else if(lift_control->lift_RC->key.v & R)
				{
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 0)//����Թ�
					{
						okok = 1;
					}
					
					if(okok == 1)
					{
						//ֹͣ����
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
					}
					else
					{
						//����
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
					}
				}
				else
				{
					//ֹͣ����
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				}
				
				//E ȡ������ PA2
				if(lift_control->lift_RC->key.v & E)
				{
					okok_delay = 0;
					okok = 0;
					//����
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
				}
				else if(okok == 1)
				{
					okok_delay++;
					if(okok_delay < 15)
					{
						//����
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
					}
					else
					{
						//ֹͣ����
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
					}
				}
				else
				{
					//ֹͣ����
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
				}
			}
			break;
		}
		case Lift_Key://�����ָ�״̬
		{
			lift_time_delay = 0;
			//ÿ4ms��һ��
			lift_keyboard_time++;
			/********************************************** �������� *******************************************************/
//			//̧��
//			if((lift_control->lift_RC->key.v == Z) && (lift_keyboard_time - lift_last_Press_time>100))
//			{
//				lift_last_Press_time = lift_keyboard_time;
//				lift_control->motor_lift[1].angle_set = 900;
//			}
//			//��λ
//			if((lift_control->lift_RC->key.v == X) && (lift_keyboard_time - lift_last_Press_time>100))
//			{
//				lift_last_Press_time = lift_keyboard_time;
//				lift_control->motor_lift[1].angle_set = 50;
//			}
//			
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;	
			
			
			/**************************************************** ��ȡ���� ****************************************************/
			//��������ÿ400msһ�Σ�״̬ǰ��
			release_delay++;
			//R 2������ PA3
			if((lift_control->lift_RC->key.v & C) && (lift_keyboard_time - lift_last_Press_time>100))	
			{
				lift_last_Press_time = lift_keyboard_time;
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3) == 1)
				{
					//��
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
					pinch_height = 750;
				}
				else
				{
					//��
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
					pinch_height = 820 ;
				}
			}
			
			//F 1���Զ�ȡ����
			if((lift_control->lift_RC->key.v & F) && (lift_keyboard_time - lift_keyboard_last_time > 300))	
			{
				lift_keyboard_last_time = lift_keyboard_time;
				if(auto_pinch == 1)
				{
					auto_pinch = 0;
				}
				else
				{
					auto_pinch = 1;
				}
			}
			
			//�Զ�ȡ��
			if(auto_pinch == 1)
			{
				auto_pinch_time++;
				if(pinch_state == 6)//����
				{
					if(release_delay > 50)
					{
						if(auto_pinch_time > 350)
						{
							pinch_state = 0;
							auto_pinch_time = 0;
							auto_pinch = 0;
						}
					}
				}
				else if(pinch_state == 5)//��
				{
					if(release_delay > 188)
					{
						if(auto_pinch_time > 300)
						pinch_state = 6;
						release_delay = 0;
					}
				}		
				else if(pinch_state == 4)//��
				{
					if(release_delay > 200)
					{
						if(auto_pinch_time > 250)
						pinch_state = 5;
						release_delay = 0;
					}
				}				
				else if(pinch_state == 3)//̧
				{
					if(auto_pinch_time > 200)
					pinch_state = 4;
					release_delay = 0;
				}				
				else if(pinch_state == 2)//��
				{
					if(auto_pinch_time > 150)
					pinch_state = 3;
				}				
				else if(pinch_state == 1)//��
				{
					if(auto_pinch_time > 100)
					pinch_state = 2;
				}				
				else if(pinch_state == 0)//ԭʼ״̬
				{
					release_delay = 0;
					if(auto_pinch_time > 50)
					pinch_state = 1;
				}
				//��������ֹ�Զ�ȡ��
				if((lift_control->lift_RC->mouse.press_l == 1) && (lift_keyboard_time - lift_last_Press_time > 200))
				{
					lift_last_Press_time = lift_keyboard_time;
					pinch_state = 0;
					auto_pinch_time = 0;
					auto_pinch = 0;
				}
			}
			
			//������
			else if((lift_control->lift_RC->mouse.press_l == 1) && (lift_keyboard_time - lift_last_Press_time > 50))
			{
				lift_last_Press_time = lift_keyboard_time;
				if(pinch_state == 6)//����
				{
					if(release_delay > 50)
					{
						pinch_state = 0;
					}
				}
				else if(pinch_state == 5)//��
				{
					if(release_delay > 188)
					{
						pinch_state = 6;
						release_delay = 0;
					}
				}		
				else if(pinch_state == 4)//��
				{
					if(release_delay > 200)
					{
						pinch_state = 5;
						release_delay = 0;
					}
				}				
				else if(pinch_state == 3)//̧
				{
					pinch_state = 4;
					release_delay = 0;
				}				
				else if(pinch_state == 2)//��
				{
					pinch_state = 3;
				}				
				else if(pinch_state == 1)//��
				{
					pinch_state = 2;
				}				
				else if(pinch_state == 0)//ԭʼ״̬
				{
					release_delay = 0;
					pinch_state = 1;
				}
			}
			
			//����Ҽ���ÿ400msһ�Σ�״̬����
			else if((lift_control->lift_RC->mouse.press_r == 1) && (lift_keyboard_time - lift_last_Press_time>100))
			{
				lift_last_Press_time = lift_keyboard_time;
				if(pinch_state == 1)//��
				{
					pinch_state = 0;//ԭʼ״̬
				}
				else if(pinch_state == 2)//��
				{
					pinch_state = 1;//��
				}				
				else if(pinch_state == 3)//̧
				{
					pinch_state = 2;//��
				}				
				else if(pinch_state == 4)//��
				{
					pinch_state = 3;//̧
				}			
				else if(pinch_state == 5)//��
				{
					pinch_state = 4;//��
				}
				else if(pinch_state == 6)//����
				{
					pinch_state = 5;//��
				}
			}
			
			//���׿���
			if(pinch_state == 1)
			{
				//1���� PC5
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				//�ɿ���ȡ PC1
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//ƽʱ�߶�
				lift_control->motor_lift[1].angle_set = pinch_height;
				//������ҩ�� PC4
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 2)
			{
				//1����
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				//��ȡ
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				//ƽʱ�߶�
				lift_control->motor_lift[1].angle_set = pinch_height;
				//������ҩ��
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 3)
			{
				//1����
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				//��ȡ
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				//̧��
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//������ҩ��
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 4)
			{
				//1����
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//��ȡ
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				//̧��
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//������ҩ��
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 5)
			{
				//1����
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//�ɿ���ȡ
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//̧��
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//������ҩ��
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 6)
			{
				//1����
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//�ɿ���ȡ
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//̧��
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//����ҩ��
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
			}
			else
			{
				//1����
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//�ɿ���ȡ
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//ƽʱ�߶�
				lift_control->motor_lift[1].angle_set = pinch_height;
				//������ҩ��
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			
			
			static uint32_t ok_delay = 0;
			static uint8_t okokokok = 0;			
			if(lift_control->motor_lift[1].angle_set > 300)
			{
				//Q ȡ������ PB0
				if(lift_control->lift_RC->key.v & Q)
				{
					//����
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				}
				//һ������
				else if(lift_control->lift_RC->key.v & R)
				{
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 0)
					{
						okokokok = 1;
					}
					
					if(okokokok == 1)
					{
						//ֹͣ����
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
					}
					else
					{
						//����
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
					}
				}
				else
				{
					//ֹͣ����
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				}
				
				//E ȡ������ PA2
				if(lift_control->lift_RC->key.v & E)
				{
					ok_delay = 0;
					okokokok = 0;
					//����
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
				}
				else if(okokokok == 1)
				{
					ok_delay++;
					if(ok_delay < 15)
					{
						//����
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
					}
					else
					{
						//ֹͣ����
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
					}
				}
				else
				{
					//ֹͣ����
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
				}
			}
			break;
		}
		default:
		{
			
		}			
	}
	
	//ȡ�������޷�
	if(lift_control->motor_lift[1].angle_set > 1200)
	{
		lift_control->motor_lift[1].angle_set = 1200;
	}
	if(lift_control->motor_lift[1].angle_set < 0)
	{
		lift_control->motor_lift[1].angle_set = 0;
	}
	
	//ƽʱΪ������ٶ�P�ϴ󣬸�λ��ʼλ��Ϊ�˱����ṹ�ٶȽ���P��С
	if(lift_control->motor_lift[1].angle_set < 200)
	{
		lift_control->motor_pos_pid[0].Kp = lift_control->motor_pos_pid[1].Kp = 3;
	}
	else
	{
		lift_control->motor_pos_pid[0].Kp = lift_control->motor_pos_pid[1].Kp = 8;
	}
	
	//����PID
	for(uint8_t i = 0; i < 2; i++)
	{
		//λ�û�
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, lift_control->motor_lift[i].angle_set);
		//�ٶȻ�
		//��λ��ʼλ��Ϊ�˱����ṹi��Ϊ0��i��������
		if(lift_control->motor_lift[i].angle_set < 50)
		{
			lift_control->motor_speed_pid[i].Ki = 0;
			lift_control->motor_speed_pid[i].Iout = 0;
		}
		else
		{
			lift_control->motor_speed_pid[i].Ki = 1;
		}
		PID_Calc(&lift_control->motor_speed_pid[i], lift_control->motor_lift[i].speed, lift_control->motor_pos_pid[i].out);
	}
	
	if(lift_control->mode == Lift_Stop)
	{
		//�ٶȻ�PID��i���������0
		lift_control->motor_speed_pid[0].Iout = lift_control->motor_speed_pid[1].Iout = 0;
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
	}
	else
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
		//Ni_Ming(0xf1,lift_control->motor_lift[1].angle,lift_control->motor_lift[0].angle,0,0);
	}
}

