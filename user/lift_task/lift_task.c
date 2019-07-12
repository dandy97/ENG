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
		//Ni_Ming(0xf1,lift_move.motor_lift[2].angle_set ,lift_move.motor_lift[2].give_current, -lift_move.motor_lift[2].angle, 0);
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
	//���µ���ٶ�
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[2].speed = lift_update->motor_lift[2].lift_motor_measure->filter_rate / 19.0f;
	
	//���µ���Ƕ�
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f;//(0~15cm)
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
	lift_update->motor_lift[2].angle = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
	
	//��ʼ������
	if(lift_mode == Init_MODE)
	{
		init_time++;
		if(init_time > 1250)
		{
			lift_mode = Rc_MODE;
			init_time = 0;
		}
	}
	if(lift_mode != Init_MODE)
	{
		//������������״̬
		switch(lift_update->lift_RC->rc.s[0])
		{
			case 1:
			{
				lift_mode = Rc_MODE;//ң��ģʽ
				break;
			}			
			case 3:
			{
				lift_mode = Key_MODE;//����ģʽ
				if(lift_update->lift_RC->rc.s[1] == 1)
				{
					pinch_mode = PINCH_RISE;
				}
				else if(lift_update->lift_RC->rc.s[1] == 3)
				{
					pinch_mode = PINCH_INIT;
				}
				else if(lift_update->lift_RC->rc.s[1] == 2)
				{
					pinch_mode = PINCH_GIVE;
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
	
	//ң���ź�ʧȥ��Ϊֹͣģʽ
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//ֹͣģʽ
	}
	else if(last_lift_mode == Stop_MODE)
	{
		lift_mode = Init_MODE;
	}
	
	last_lift_mode = lift_mode;
}

//��������PID����
void lift_control_loop(lift_move_t *lift_control)
{	
	switch(lift_mode)
	{
		case Stop_MODE://ֹͣ״̬
		{
			//����λ������
			lift_control->motor_lift[2].angle_set = lift_control->motor_lift[2].angle;
			break;
		}
		case Rc_MODE://ң���ָ�״̬
		{
			//����λ������
			lift_control->motor_lift[0].angle_set += lift_control->lift_RC->rc.ch[3] * 0.0005f;		
			//lift_control->motor_lift[2].speed_set = -lift_control->lift_RC->rc.ch[2] * 0.0454f;		
			lift_control->motor_lift[2].angle_set = -0.0f;
			break;
		}
		case Key_MODE://����ģʽ
		{
				switch(pinch_mode)
				{
					case PINCH_INIT://��ʼ״̬
					{
						//����λ������
						lift_control->motor_lift[0].angle_set = 2.0f;
						break;
					}
					case PINCH_RISE://����
					{
						//����λ������
						lift_control->motor_lift[0].angle_set = 10.0f;
						break;
					}
					case PINCH_GIVE://����ģʽ
					{
						//����λ������
						lift_control->motor_lift[0].angle_set = 12.0f;	
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
	if(lift_control->motor_lift[0].angle_set > 15.0f)
	{
		lift_control->motor_lift[0].angle_set = 15.0;
	}
	else if(lift_control->motor_lift[0].angle_set < 2.01f)
	{
		lift_control->motor_lift[0].angle_set = 2.0;
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
	
	//ƽ�Ƽ���PID
	//PID_Calc(&lift_control->motor_speed_pid[2], lift_control->motor_lift[2].speed, lift_control->motor_lift[2].speed_set);
	
	if(lift_mode == Stop_MODE)
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current  = lift_control->motor_lift[2].give_current = 0;
	}
	else
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
		lift_control->motor_lift[2].give_current = lift_control->motor_speed_pid[2].out;
	}
}

//��������״̬
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

