#include "chassis_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

#include "usart1.h"
#include "rc.h"
#include "tof.h"
#include "put_out.h"

#include "can_receive.h"
#include "pid.h"
#include "ramp.h"

//��ʼ�����̼����ٶ�б�º���
static ramp_t FBSpeedRamp = RAMP_GEN_DAFAULT;
static ramp_t LRSpeedRamp = RAMP_GEN_DAFAULT;

static chassis_mode_e chassis_mode = INIT_MODE;		

extern float forward_back_speed, left_right_speed;		
//static float pidin, pp, pi, pd, spp, si, sd = 0;	
//�������ݽṹ��
chassis_move_t chassis_move;
//��������ռ�ʣ����
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
	//����һ��ʱ��
  vTaskDelay(2000);
	//���̳�ʼ��
	chassis_init(&chassis_move);
	static uint32_t send_lift_wheel = 0;
	while(1)
	{
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
		//���̿���PID����
		chassis_control_loop(&chassis_move);
		//����������ѭ��
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,	chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		
		//��������Ƶ��4ms	 
		if((send_lift_wheel++) % 4 == 0)
		{
			CAN_CMD_CHASSIS_LIFT(chassis_move.motor_chassis[4].give_current, chassis_move.motor_chassis[5].give_current,0,0);
			//Ni_Ming(0xf1, chassis_move.pinch_dis_l, chassis_move.pinch_dis_r, chassis_move.climb_dis_r, chassis_move.climb_dis_l);
			//printf("%d %d\r\n",CLIMB_FOR_STATE, CLIMB_BACK_STATE);
		}
		
		vTaskDelay(2);
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//���̳�ʼ��
void chassis_init(chassis_move_t *chassis_init)
{
	if (chassis_init == NULL)
	{
		return;
	}
	
	//�����ٶȻ�PIDֵ
	const static float motor_speed_pid[3] = {700, 0, 0};
	//����λ�û�PIDֵ
	const static float motor_pos_pid[3] = {0, 0, 0};
	
	//��ȡ���̵������ָ��
	for (uint8_t i = 0; i < 6; i++)
	{  
		chassis_init->motor_chassis[i].chassis_motor_measure = get_Motor_Measure_Point(i);
	}
	//��ȡң��ָ�� 
	chassis_init->chassis_RC = get_remote_control_point();
	
	//��ȡ����������ָ��
	chassis_init->gyro_data = get_GYRO_Measure_Point();
	
	//��ȡ����tof����
	chassis_init->tof_measure = get_tof_Info_Measure_Point();
	
	//��ȡ�ǵ�ȡ��tof����
	chassis_init->tof_can_measure = get_tof_can_Info_Measure_Point();
	
	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 6; i++)
	{
		//pmax imax
		PID_Init(&chassis_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, 10000, 0);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i= 0; i < 6; i++)
	{
		//pmax imax
		PID_Init(&chassis_init->motor_pos_pid[i], PID_POSITION, motor_pos_pid, 0, 0);
	}
	 
	//��ʼ��Z��PID
	CHISSIS_PID_Init(&chassis_init->chassis_gryo_pid, 2000, 0, 0.02, 0, 0);//kp_out ki_out kp ki kd 20 60
	CHISSIS_PID_Init(&chassis_init->chassis_acc_pid, 60, 0, 200, 0, 300);
	
	//���������ʼ�����ʱ���������ǵĽǶ�
	chassis_init->gyro_angle_start = chassis_init->gyro_data->yaw;
	
	//����һ������
  chassis_feedback_update(chassis_init);
}

//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_update)
{
	//���µ���ٶȣ����ٶ����ٶȵ�PID΢��
	chassis_update->motor_chassis[0].speed = chassis_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[1].speed = chassis_update->motor_chassis[1].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[2].speed = chassis_update->motor_chassis[2].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[3].speed = chassis_update->motor_chassis[3].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[4].speed = chassis_update->motor_chassis[4].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[5].speed = chassis_update->motor_chassis[5].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->vw_mouse = chassis_update->chassis_RC->mouse.x;
	chassis_update->vy_mouse += chassis_update->chassis_RC->mouse.y * 0.05f;
	chassis_update->tof_h = chassis_update->tof_measure->tof_h;
	chassis_update->climb_dis_l = chassis_update->tof_can_measure->climb_dis_l;
	chassis_update->climb_dis_r = chassis_update->tof_can_measure->climb_dis_r;
	chassis_update->pinch_dis_l = chassis_update->tof_can_measure->pinch_dis_l;
	chassis_update->pinch_dis_r = chassis_update->tof_can_measure->pinch_dis_r;
	chassis_update->yaw = chassis_update->gyro_data->yaw;
	
	//��ȡ������������
	chassis_update->last_gyro_heartbeat = chassis_update->gyro_heartbeat;
	chassis_update->gyro_heartbeat = get_gyro_heartbeat();
	
	//���µ���״̬
	switch(chassis_update->chassis_RC->rc.s[0])
	{
		case 1:
		{
			chassis_mode = RC_MODE;
			break;
		}			
		case 3:
		{
			chassis_mode = KEY_MODE;
			break;
		}
		case 2:
		{
			chassis_mode = STOP_MODE;
			chassis_update->gyro_angle_start = chassis_update->gyro_data->yaw;
			chassis_update->vw_offset = 0;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//ʧȥң���źź����ģʽΪֹͣģʽ
	if(xTaskGetTickCount() - chassis_update->chassis_RC->time > 88)
	{
		chassis_mode = STOP_MODE;
		chassis_update->gyro_angle_start = chassis_update->gyro_data->yaw;
		chassis_update->vw_offset = 0;
	}
}

//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_control)
{
	switch(chassis_mode)
	{
		case RC_MODE://ң��ģʽ
		{
			chassis_control->vx =  chassis_control->chassis_RC->rc.ch[0] * 60.0f/660.0f;
			chassis_control->vy =  chassis_control->chassis_RC->rc.ch[1] * 60.0f/660.0f;
			chassis_control->vw_offset -= chassis_control->chassis_RC->rc.ch[2] * 50/660 *  0.02;
			chassis_control->vw_set = chassis_control->vw_offset + chassis_control->gyro_angle_start;
			break;
		}			
		case KEY_MODE://����ģʽ
		{
			chassis_control->key_time++;//4msһ��
			
			/**********************************************  �ٶȵ�*******************************************************************/		
			if(chassis_control->auto_count == 2)
			{
				chassis_control->vy_offset = 0;
				chassis_control->vx_offset = 0;
			}
			else if(HAL_GPIO_ReadPin(CLIMB_FOR))
			{
				chassis_control->vy_offset = 15;
				chassis_control->vx_offset = 0;
			}
			else if(chassis_control->chassis_RC->rc.s[1] == 1)
			{
				chassis_control->vy_offset = 30;
				chassis_control->vx_offset = 20;
			}
			else if(chassis_control->chassis_RC->key.v & SHIFT)//shitf����																
			{
				chassis_control->vy_offset = 60;
				chassis_control->vx_offset = 50;
			}
			else
			{
				chassis_control->vy_offset = 50;
				chassis_control->vx_offset = 40;
			}
			/**********************************************  �ٶȵ�*******************************************************************/
			
	   	/**********************************************  �ƶ�  *******************************************************************/
			
			//W��Sǰ��
			if(chassis_control->chassis_RC->key.v & W)
			{
				chassis_control->vy = chassis_control->vy_offset * ramp_calc(&FBSpeedRamp);
			}
			else if(chassis_control->chassis_RC->key.v & S)
			{
				chassis_control->vy = -chassis_control->vy_offset * ramp_calc(&FBSpeedRamp);
			}
			else
			{
				chassis_control->vy =0;
				ramp_init(&FBSpeedRamp, 400);
			}
		
			//A��Dƽ��
			if(chassis_control->chassis_RC->key.v & A)
			{
				chassis_control->vx = -chassis_control->vx_offset * ramp_calc(&LRSpeedRamp);
			}
			else if(chassis_control->chassis_RC->key.v & D)
			{
				chassis_control->vx = chassis_control->vx_offset * ramp_calc(&LRSpeedRamp);
			}
			else
			{
				chassis_control->vx = 0;
				ramp_init(&LRSpeedRamp, 400);
			}
			
			if(chassis_control->chassis_RC->rc.s[1] == 3)
			{
				//�Զ��ǵ�
				if((chassis_control->chassis_RC->key.v & R) && (chassis_control->key_time - chassis_control->last_press_time >500))
				{
					chassis_control->last_press_time = chassis_control->key_time;
					chassis_control->auto_climb = 1;
				}
				else if(chassis_control->chassis_RC->mouse.press_r == 1)//����Ҽ�ȡ���Զ��ǵ�
				{
					chassis_control->auto_climb = 0;
				}
			}
			
			static uint32_t lajidaima = 0;
			if(chassis_control->auto_climb == 1)//�Զ��ǵ�
			{
				if(chassis_control->auto_count == 0)
				{
					Control_Gpio(CLIMB_FOR, 1);
					chassis_control->auto_count = 1;
					lajidaima = 0;
				}
				if(chassis_move.climb_dis_l < 7.5f && chassis_move.climb_dis_r < 7.5f && CLIMB_FOR_STATE && chassis_control->auto_count == 1)
				{
					lajidaima++;
					if(lajidaima > 100)
					{
						lajidaima = 0;
						Control_Gpio(CLIMB_BACK, 1);
						chassis_control->auto_count = 2;
					}
				}
				else if(CLIMB_BACK_STATE && chassis_control->auto_count == 2)
				{
					lajidaima++;
					Control_Gpio(CLIMB_FOR, 0);
					if(lajidaima > 20)
					{
						lajidaima = 0;
						chassis_control->auto_count = 3;
					}
				}
				else if(chassis_control->auto_count == 3 && chassis_move.tof_h < 20.0f)
				{
					Control_Gpio(CLIMB_BACK, 0);
					chassis_control->auto_count = 0;
					chassis_control->auto_climb = 0;
				}
			}
			else
			{
				chassis_control->auto_count = 0;
			}
			
			//��ת
			if(chassis_control->chassis_RC->rc.s[1] == 1)//ȡ��״̬ ����������ת
			{
				chassis_control->vw_offset = 0;
				chassis_control->vw_set = chassis_control->gyro_data->yaw;
				chassis_control->gyro_angle_start = chassis_control->gyro_data->yaw;
				if(chassis_control->chassis_RC->key.v & R)//һ����λ
				{
					if(chassis_control->pinch_dis_l < 19.0f && chassis_control->pinch_dis_r < 19.0f)
					{
						chassis_control->vy = 0.0f;
					}
					else if(chassis_control->pinch_dis_l > 16.0f || chassis_control->pinch_dis_r > 16.0f)
					{
						chassis_control->vy = 25.0f;
					}
				}
			}
			else if(HAL_GPIO_ReadPin(CLIMB_FOR))//����ǰ��
			{
				chassis_control->vw_offset = 0;
				chassis_control->vw_set = chassis_control->gyro_data->yaw;
				chassis_control->gyro_angle_start = chassis_control->gyro_data->yaw;
			}
			else if((chassis_control->chassis_RC->key.v & CTRL) && (chassis_control->key_time - chassis_control->last_press_time >500) && (chassis_control->chassis_RC->rc.s[1] != 1))//ת180
			{
				chassis_control->gyro_angle_start = chassis_control->gyro_data->yaw;
				chassis_control->last_press_time = chassis_control->key_time;
				chassis_control->vw_offset += 180;//ת180
			}
			else //������
			{
				if(chassis_control->vw_mouse >  60)chassis_control->vw_mouse = 60; 
				if(chassis_control->vw_mouse < -60)chassis_control->vw_mouse = -60;			
				chassis_control->vw_offset -= chassis_control->vw_mouse * 0.003;
				chassis_control->vw_set = chassis_control->vw_offset + chassis_control->gyro_angle_start;
			}
			
			/**********************************************  �ƶ�  *******************************************************************/
			
			/**********************************************���׿���*******************************************************************/
			
			if(chassis_control->chassis_RC->rc.s[1] == 1)//ȡ��ģʽ
			{
				Set_KEY_GPIO(chassis_control->chassis_RC->key.v, G, OPEN_LID, chassis_control->key_time);//����
			}
			else if(chassis_control->chassis_RC->rc.s[1] == 2)//����ģʽ
			{
				Set_KEY_GPIO(chassis_control->chassis_RC->key.v, G, OPEN_LID, chassis_control->key_time);//����
			}
			else
			{
				Set_KEY_GPIO(chassis_control->chassis_RC->key.v, Q, CLIMB_FOR, chassis_control->key_time);//ǰ�ǵ���
				Set_KEY_GPIO(chassis_control->chassis_RC->key.v, E, CLIMB_BACK, chassis_control->key_time);//��ǵ���
				Set_KEY_GPIO(chassis_control->chassis_RC->key.v, Z, TRAILER, chassis_control->key_time);//�ϳ�
			}
			
			//printf("%d\r\n",chassis_control->chassis_RC->key.time);
			/**********************************************���׿���*******************************************************************/
			break;
		}
		case STOP_MODE://ֹͣģʽ
		{
			chassis_control->vx = chassis_control->vy = 0;
			chassis_control->vw_set = chassis_control->gyro_data->yaw;
			chassis_control->gyro_angle_start = chassis_control->gyro_data->yaw;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//Z��Ƕ�PID����
	PID_Calc(&chassis_control->chassis_gryo_pid, chassis_control->yaw, chassis_control->vw_set);
	//Z����ٶ�PID����	
	chassis_control->vw = PID_Calc(&chassis_control->chassis_acc_pid, -chassis_control->gyro_data->v_z, -chassis_control->chassis_gryo_pid.out);
	
	static uint32_t lost_time = 0;
	//���������ݶ�ʧ��
	if(chassis_control->gyro_heartbeat == chassis_control->last_gyro_heartbeat)
	{
		lost_time++;
		if(lost_time > 200)
		{
			chassis_control->vw = 0;			
//			if(chassis_control->vw_mouse > 0)
//			{
//				chassis_control->vw = 20;
//			}
//			else if(chassis_control->vw_mouse < 0)
//			{
//				chassis_control->vw = -20;
//			}
//			chassis_control->vw = chassis_control->vw_mouse;
		}
	}
	else
	{
		lost_time = 0;
	}
	//�����ٶ��趨
	chassis_control->motor_chassis[0].speed_set = +(int16_t)chassis_control->vx - (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[1].speed_set = +(int16_t)chassis_control->vx + (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[2].speed_set = -(int16_t)chassis_control->vx - (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[3].speed_set = -(int16_t)chassis_control->vx + (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[4].speed_set = -(int16_t)chassis_control->vy;
	chassis_control->motor_chassis[5].speed_set = (int16_t)chassis_control->vy;
	
	//����PID
	PID_Calc(&chassis_control->motor_speed_pid[0], chassis_control->motor_chassis[0].speed, chassis_control->motor_chassis[0].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[1], chassis_control->motor_chassis[1].speed, chassis_control->motor_chassis[1].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[2], chassis_control->motor_chassis[2].speed, chassis_control->motor_chassis[2].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[3], chassis_control->motor_chassis[3].speed, chassis_control->motor_chassis[3].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[4], chassis_control->motor_chassis[4].speed, chassis_control->motor_chassis[4].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[5], chassis_control->motor_chassis[5].speed, chassis_control->motor_chassis[5].speed_set);
	
	//��ֵ����ֵ
	chassis_control->motor_chassis[0].give_current = (int16_t)(chassis_control->motor_speed_pid[0].out);
	chassis_control->motor_chassis[1].give_current = (int16_t)(chassis_control->motor_speed_pid[1].out);
	chassis_control->motor_chassis[2].give_current = (int16_t)(chassis_control->motor_speed_pid[2].out);
	chassis_control->motor_chassis[3].give_current = (int16_t)(chassis_control->motor_speed_pid[3].out);
	chassis_control->motor_chassis[4].give_current = (int16_t)(chassis_control->motor_speed_pid[4].out);
	chassis_control->motor_chassis[5].give_current = (int16_t)(chassis_control->motor_speed_pid[5].out);
	
	TIM4->CCR4 = 1050 + chassis_control->vy_mouse;
	if(chassis_control->vy_mouse > 750)
	{
		 chassis_control->vy_mouse = 750;
	}
	else if(chassis_control->vy_mouse < 0)
	{
		 chassis_control->vy_mouse = 0;
	}
		//printf("%f\r\n",chassis_control->vy_mouse);
}

//���ص�������״̬
uint8_t get_chassis_state(void)
{
	return chassis_mode;
}

//����Z��PID��ʼ��
void CHISSIS_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
	if (pid == NULL)
	{
			return;
	}
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	pid->set = 0.0f;

	pid->max_iout = max_iout;
	pid->max_out = maxout;
}

//��ȡ������
uint8_t get_heartbeat_bag(uint32_t data)
{
	static uint32_t count;
	static uint32_t last_data;
	if(data == last_data)
	{
		count++;
	}
	else
	{
		count = 0;
	}
	last_data = data;
	if(count > 5)
	  return 0;
	else
		return 1;
}

//IO�ڰ������Ƹߵ͵�ƽ
void Set_KEY_GPIO(uint16_t key, uint16_t key1, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t time)
{
	static uint32_t last_time = 0;
	if((key & key1) && (time - last_time >250))
	{
		last_time = time;
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
