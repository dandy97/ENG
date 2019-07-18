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
#include "put_out.h"

// ��������������������0-IDΪ7 �ұ��������1-IDΪ8

//��������״̬
lift_mode_e lift_mode = Init_MODE;
lift_mode_e last_lift_mode = Init_MODE;
pinch_mode_e pinch_mode = Pinch_Init;
pinch_mode_e last_pinch_mode = Pinch_Init;

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
		//printf("%d",Flip_State);
		//Ni_Ming(0xf1,lift_move.motor_lift[0].angle ,lift_move.motor_lift[2].angle_set, 0, 0);
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
	
	//ƽ���ٶ�λ�û�
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
	PID_Init(&lift_init->motor_pos_pid[2], PID_POSITION, translation_pos_pid, 60, 0);
	PID_Init(&lift_init->motor_speed_pid[2], PID_POSITION, translation_speed_pid, 10000, 3000);
	
	//����һ������
  lift_feedback_update(lift_init);
}


//�������ݸ���
void lift_feedback_update(lift_move_t *lift_update)
{	
	//��ʼ������
	if(lift_mode == Init_MODE)
	{
		static uint32_t init_time = 0;
		init_time++;
		if(init_time > 1250 || !HAL_GPIO_ReadPin(Limit_Switch))
		{
			init_time = 0;
			lift_mode = Ready_MODE;
			lift_update->lift_right_cail  = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->lift_left_cail   = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
			lift_update->translation_cail = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
		}
	}
	//׼������ ƽ�ƹ���
	if(lift_mode == Ready_MODE || lift_mode == Reset_MODE)
	{
		if(lift_update->motor_lift[2].angle > 29 && lift_update->motor_lift[2].angle < 40)
		{
			lift_mode = Start_MODE;
		}
	}
	//��ʼģʽ
	if(lift_mode == Start_MODE)
	{
		if(lift_update->lift_RC->rc.s[0] == 3 && lift_update->lift_RC->rc.s[1] == 1)
		{
			pinch_mode = Pinch_Key;
		}
		else
		{
			pinch_mode = Pinch_Init;
		}
	}
	
	//ң���ź�ʧȥ��Ϊֹͣģʽ
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//ֹͣģʽ
		pinch_mode = Pinch_Init;
	}
	else if(last_lift_mode == Stop_MODE)
	{
		lift_mode = Init_MODE;
	}
	
	//����״̬��
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

static uint8_t high_control = 0;
//��������PID����
void lift_control_loop(lift_move_t *lift_control)
{	
	lift_control->key_time++;
	/**********************************************�������*******************************************************************/	
	switch(lift_mode)
	{
		case Init_MODE:
		{
			lift_control->motor_lift[2].speed_set = -25;
			break;
		}			
		case Ready_MODE:
		{
			lift_control->motor_lift[2].angle_set = -35.5f;
			break;
		}		
		case Start_MODE:
		{
			if(lift_control->lift_RC->rc.s[0] == 1)//ң��������ϵ�λ
			{
				high_control = 0;
				if(Extend_State == 1)lift_control->motor_lift[0].angle_set += lift_control->lift_RC->rc.ch[3] * 0.0005;
				lift_control->motor_lift[2].angle_set = -35.5f;
			}
			else if(lift_control->lift_RC->rc.s[0] == 3)//ң������м䵵λ
			{
				if(lift_control->lift_RC->rc.s[1] == 1)//ң���ұ����ϵ�λ
					{	
						if(high_control == 1)
						{
							lift_control->motor_lift[0].angle_set = 18.0f;
							if(lift_control->motor_lift[0].angle < -16.8f)
							{
								lift_control->cylinder_state.extend = 1;//���							
							}
						}
						else if(high_control == 0)
						{
							lift_control->cylinder_state.extend = 0;
							if(Extend_State == 1)
							lift_control->motor_lift[0].angle_set = 14.0f;
						}
						//�Զ�ȡ�� Qȡ3�� Eȡ5�� Fȡһ��
						if((lift_control->lift_RC->key.v & Q) && (lift_control->key_time - lift_control->last_key_time >200))
						{
							lift_control->last_key_time = lift_control->key_time;
							lift_control->auto_mode = 1;
						}
						if((lift_control->lift_RC->key.v & E) && (lift_control->key_time - lift_control->last_key_time >200))
						{
							lift_control->last_key_time = lift_control->key_time;
							lift_control->auto_mode = 2;
						}
						if((lift_control->lift_RC->key.v & F) && (lift_control->key_time - lift_control->last_key_time >200))
						{
							lift_control->last_key_time = lift_control->key_time;
							lift_control->auto_mode = 3;
						}
						if((lift_control->lift_RC->key.v & C) && (lift_control->key_time - lift_control->last_key_time >200))
						{
							lift_control->last_key_time = lift_control->key_time;
							if(high_control == 1)
							{
								high_control = 0;//����
							}
						  else if(high_control == 0)
							{
								high_control = 1;//���
							}
						}
						//�һ�ȡ��
						if(lift_control->lift_RC->mouse.press_r == 1)
						{
							lift_control->auto_mode = 0;		
							high_control = 0;
						}
						//�Զ�ȡ��
						Auto_Mvp(&lift_control->cylinder_state, &lift_control->auto_mode, &lift_control->motor_lift[2].angle, &lift_control->motor_lift[2].angle_set, &lift_control->motor_lift[0].angle_set);
				}
				else if(lift_control->lift_RC->rc.s[1] == 3)//ң������м䵵λ
				{
						high_control = 0;
						if(lift_control->motor_lift[2].angle > 30 && lift_control->motor_lift[2].angle < 40 && Extend_State)lift_control->motor_lift[0].angle_set = 1.0f;
						lift_control->motor_lift[2].angle_set = -35.5f;
				}
				else if(lift_control->lift_RC->rc.s[1] == 2)//ң��������µ�λ
				{
						lift_control->motor_lift[0].angle_set = 18.0f;
						lift_control->motor_lift[2].angle_set = -35.5f;
				}
			}
			break;
		}		
		case Stop_MODE:
		{
			if(Extend_State == 1)lift_control->motor_lift[0].angle_set = 0;
			lift_control->motor_lift[2].speed_set = 0.0f;
			high_control = 0;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//�����߶��޷�(2cm ~ 15cm)
	if(lift_control->motor_lift[0].angle_set > 19.0f)
	{
		lift_control->motor_lift[0].angle_set = 19.0;
	}
	else if(lift_control->motor_lift[0].angle_set < 1.01f)
	{
		lift_control->motor_lift[0].angle_set = 1.0;
	}
	
	//ƽ�ƾ����޷�
	if(lift_control->motor_lift[2].angle_set < -69.1f)
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
	
	if(lift_mode == Stop_MODE || lift_mode == Init_MODE)//ֹͣ״̬
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
	

	/**********************************************���׿���*******************************************************************/
	if(pinch_mode == Pinch_Key)
	{
		Control_Gpio(EXTEND,lift_control->cylinder_state.extend);
		Control_Gpio(PINCH,lift_control->cylinder_state.pinch);
		Control_Gpio(FLIP,lift_control->cylinder_state.flip);
		Control_Gpio(BOUNCE,lift_control->cylinder_state.bounce);
	}
	else
	{
		Control_Gpio(EXTEND,0);
		Control_Gpio(PINCH,0);
		Control_Gpio(FLIP,0);
		Control_Gpio(BOUNCE,0);
	}
	/**********************************************���׿���*******************************************************************/
}

//��������״̬
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

static uint8_t auto_mode = 0;
void Auto_Mvp(Cylinder_State_t *cylinde_state, uint8_t *mode, float *angle, float *angle_set, float *high_angle_set)
{
	static uint8_t auto_times, box_times, mmp = 1;
	static uint32_t auto_time = 0;
	if(*mode == 0)//��λ
	{
		cylinde_state->flip = 0;
		cylinde_state->pinch = 0;
		cylinde_state->bounce = 0;
		box_times = 1;
		*angle_set = -35.5f;
		auto_times = 0;
		auto_mode = 0;
		mmp = 1;
		Auto_Auto_Auto(cylinde_state, &auto_times);
	}
	else if(*mode == 1)//ȡ����
	{
		if(box_times == 1)
		{
			if(auto_mode < 4 && (*angle_set == -35.5f))
			{
				*angle_set = -35.5f;
			}
			else
			{	
				*angle_set = -68.5f;
			}
			
			if(*angle > 30 && *angle < 40)
			{
				if(mmp == 1)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 60 && *angle < 70)
			{
				if(auto_times == 0)
				{
					box_times = 2;
				}			
			}
			Auto_Auto_Auto(cylinde_state, &auto_times);
		}
		else if(box_times == 2)
		{
			if(auto_mode < 4 && (*angle_set == -68.5f))
			{
				*angle_set = -68.5f;
			}
			else
			{	
				*angle_set = -5.0f;
			}
			
			if(*angle > 60 && *angle < 70)
			{
				if(mmp == 2)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 0 && *angle < 10)
			{
				if(auto_times == 0)
				{
					box_times = 3;
				}			
			}
			Auto_Auto_Auto(cylinde_state, &auto_times);
		}
		else if(box_times == 3)
		{
			if(auto_mode < 4 && (*angle_set == -5.0f))
			{
				*angle_set = -5.0f;
			}
			else
			{	
				*angle_set = -35.5f;
			}
			
			if(*angle > 0 && *angle < 10)
			{
				if(mmp == 3)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 30 && *angle < 40)
			{
				if(auto_times == 0)
				{
					box_times = 4;
				}			
			}
			Auto_Auto_Auto(cylinde_state, &auto_times);
		}
		else if(box_times == 4)
		{
			*angle_set = -35.5f;
			high_control = 0;
			if(*angle > 30 && *angle < 40)
			{
				*mode = 0;
				box_times = 1;
			}
		}
	}
	else if(*mode == 2)//ȡ���
	{
		if(box_times == 1)
		{
			if(auto_mode < 4 && (*angle_set == -35.5f))
			{
				*angle_set = -35.5f;
			}
			else
			{	
				*angle_set = -68.5f;
			}
			
			if(*angle > 30 && *angle < 40)
			{
				if(mmp == 1)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 60 && *angle < 70)
			{
				if(auto_times == 0)
				{
					box_times = 2;
				}			
			}
			Auto_Auto_Auto(cylinde_state, &auto_times);
		}
		else if(box_times == 2)
		{
			if(auto_mode < 4 && (*angle_set == -68.5f))
			{
				*angle_set = -68.5f;
			}
			else
			{	
				*angle_set = -5.0f;
			}
			
			if(*angle > 60 && *angle < 70)
			{
				if(mmp == 2)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 0 && *angle < 10)
			{
				if(auto_times == 0)
				{
					box_times = 3;
				}			
			}
			Auto_Auto_Auto(cylinde_state, &auto_times);
		}
		else if(box_times == 3)
		{
			if(auto_mode < 4 && (*angle_set == -5.0f))
			{
				*angle_set = -5.0f;
			}
			else
			{	
				*angle_set = -20.5f;
			}
			
			if(*angle > 0 && *angle < 10)
			{
				if(mmp == 3)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 15 && *angle < 25)
			{
				if(auto_times == 0)
				{
					box_times = 4;
				}			
			}
			Auto_Auto_Auto(cylinde_state, &auto_times);
		}
		else if(box_times == 4)//��4��
		{
			high_control = 1;
			if(auto_mode < 4 && (*angle_set == -20.5f))
			{
				*angle_set = -20.5f;
			}
			else
			{	
				*angle_set = -50.5f;
			}
			
			if(*angle > 15 && *angle < 25)
			{
				if(mmp == 4)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 45 && *angle < 55)
			{
				if(auto_times == 0)
				{
					box_times = 5;
				}			
			}
			
			static uint32_t aaaa = 0;
			if(Extend_State == 0)
			{			
				aaaa++;
				if(aaaa > 100)
				Auto_Auto_Auto(cylinde_state, &auto_times);
			}		
			else
			{
				aaaa = 0;
			}
		}
		else if(box_times == 5)//��5��
		{
			high_control = 1;
			if(auto_mode < 4 && (*angle_set == -50.5f))
			{
				*angle_set = -50.5f;
			}
			else
			{	
				*angle_set = -35.5f;
			}
			
			if(*angle > 45 && *angle < 55)
			{
				if(mmp == 5)
				{
					mmp++;
					auto_times = 1;
				}
			}
			
			if(*angle > 30 && *angle < 40)
			{
				if(auto_times == 0)
				{
					box_times = 6;
				}			
			}
			static uint32_t aaaa = 0;
			if(Extend_State == 0)
			{			
				aaaa++;
				if(aaaa > 50)
				Auto_Auto_Auto(cylinde_state, &auto_times);
			}		
			else
			{
				aaaa = 0;
			}
		}
		else if(box_times == 6)
		{
			high_control = 0;
			*angle_set = -35.5f;
			if(*angle > 30 && *angle < 40)
			{
				*mode = 0;
				box_times = 1;
			}
		}
	}
	//ȡһ��
	else if(*mode == 3)
	{
		auto_times = 1;
		Auto_Auto_Auto(cylinde_state, &auto_times);
		if(auto_times == 0)
		{
			*mode = 0;
		}
	}
}

//�Զ�ȡ��
void Auto_Auto_Auto(Cylinder_State_t *cylinde_state, uint8_t *mode)
{
	static uint32_t auto_delay_time = 0;
	if(*mode == 1)
	{
		if(auto_mode == 0)//��ʼ״̬
		{
			auto_delay_time++;
			cylinde_state->flip = 0;
			cylinde_state->pinch = 0;
			cylinde_state->bounce = 0;
			if(!Flip_State && !Pinch_State && !Bounce_State)
			{
				auto_mode = 1;
				auto_delay_time = 0;
			}
		}
		else if(auto_mode == 1)//����ȥ
		{
			auto_delay_time++;
			cylinde_state->flip = 1;
			cylinde_state->pinch = 0;
			cylinde_state->bounce = 0;
			if((Flip_State && !Pinch_State && !Bounce_State) || auto_delay_time >500)
			{
				auto_mode = 2;
				auto_delay_time = 0;
			}
		}
		else if(auto_mode == 2)//����ȥ->��ס
		{
			auto_delay_time++;
			cylinde_state->flip = 1;
			cylinde_state->pinch = 1;
			cylinde_state->bounce = 0;
			if((Flip_State && Pinch_State && !Bounce_State) || auto_delay_time > 100)
			{
				auto_mode = 3;
				auto_delay_time = 0;
			}
		}
		else if(auto_mode == 3)//����ȥ->��ס->������
		{
			auto_delay_time++;
			cylinde_state->flip = 0;
			cylinde_state->pinch = 1;
			cylinde_state->bounce = 0;
			if(auto_delay_time > 150)
			{
				auto_mode = 4;
				auto_delay_time = 0;
			}
		}
		else if(auto_mode == 4)//����ȥ->��ס->������->�ɵ�
		{
			auto_delay_time++;
			cylinde_state->flip = 0;
			cylinde_state->pinch = 1;
			cylinde_state->bounce = 0;
			if(auto_delay_time > 100)
			{
				auto_mode = 5;
				auto_delay_time = 0;
			}
		}
		else if(auto_mode == 5)//��λ
		{
			auto_delay_time++;
			cylinde_state->flip = 0;
			cylinde_state->pinch = 0;
			cylinde_state->bounce = 1;
			if((!Flip_State && !Pinch_State && Bounce_State) || auto_delay_time > 100)
			{
				auto_delay_time = 0;
				auto_mode = 0;
				*mode = 0;
			}		
		}
	}
	else
	{
		auto_mode = 0;
	}
}

