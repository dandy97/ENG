#include "chassis_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

#include "usart1.h"
#include "rc.h"
#include "tof.h"

#include "can_receive.h"
#include "pid.h"
#include "ramp.h"
//#include "user_lib.h"
//#include "math.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//��ʼ�����̼����ٶ�б�º���
static ramp_t FBSpeedRamp = RAMP_GEN_DAFAULT;
static ramp_t LRSpeedRamp = RAMP_GEN_DAFAULT;

extern float forward_back_speed, left_right_speed;		
		
//�������ݽṹ��
chassis_move_t chassis_move;

//��������ռ�ʣ����
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
//	static float pid_in,pid_kp,pid_ki,pid_kd = 0;
	//����һ��ʱ��
  vTaskDelay(2000);
	//���̳�ʼ��
	chassis_init(&chassis_move);
	while(1)
	{
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
		//���̿���PID����
		chassis_control_loop(&chassis_move);
		//����������ѭ��
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,	chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		//��������Ƶ��4ms
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	 
		vTaskDelay(4);
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
	const static float motor_speed_pid[3] = {500, 0, 0};
	//����λ�û�PIDֵ
	const static float motor_pos_pid[3] = {0, 0, 0};
	
	//��ȡ���̵������ָ��
	for (uint8_t i = 0; i < 4; i++)
	{  
		chassis_init->motor_chassis[i].chassis_motor_measure = get_Motor_Measure_Point(i);
	}
	
	//��ȡң��ָ�� 
	chassis_init->chassis_RC = get_remote_control_point();
	
	//��ȡ����������ָ��
	chassis_init->gyro_data = get_GYRO_Measure_Point();
	
	//��ȡ����tof����
	chassis_init->tof_measure = get_tof_Info_Measure_Point();
	
	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 4; i++)
	{
		//pmax imax
		PID_Init(&chassis_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, 10000, 0);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i = 0; i < 4; i++)
	{
		//pmax imax
		PID_Init(&chassis_init->motor_pos_pid[i], PID_POSITION, motor_pos_pid, 0, 0);
	}
	
	//���������ʼ�����ʱ���������ǵĽǶ�
	chassis_init->gyro_angle_start = chassis_init->gyro_data->yaw;
	
	//��ʼ�������̨�Ƕ�
	chassis_init->gimbal_y_offset = 2250;
	//����һ������
  chassis_feedback_update(chassis_init);
}

//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_update)
{
	//��1��ң���жϿ�ʼ��ʱ
	start_chassis_task = 1;
	//���µ���ٶȣ����ٶ����ٶȵ�PID΢��
	chassis_update->motor_chassis[0].speed = chassis_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[1].speed = chassis_update->motor_chassis[1].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[2].speed = chassis_update->motor_chassis[2].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[3].speed = chassis_update->motor_chassis[3].chassis_motor_measure->filter_rate / 19.0f;
	
	chassis_update->tof_h = chassis_update->tof_measure->tof_h;
}


//��������ǰϵͳʱ��
static uint32_t chassis_system_time = 0;
//����ģʽʱ�� ��һ�ΰ��°���ʱ��
static uint32_t keyboard_time, last_Press_time = 0;
//����ǰ�ֺ��ֱ�־λ
static uint8_t ofwheel, obwheel, tuo_che = 0;
//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_control)
{
	//ң���ұ߲����м䵲Ϊ����ģʽ
	if(chassis_control->chassis_RC->rc.s[0] == 3)
	{
		//ÿ4ms��һ��
		keyboard_time++;
		if(chassis_control->chassis_RC->key.v & SHIFT)																	
		{
			forward_back_speed = 60;
			left_right_speed 	 = 50;
		}
		else if(ofwheel == 1)//ǰ�ִ���
		{
			forward_back_speed = 15;
			left_right_speed 	 = 30;
		}
		else if(obwheel == 1)
		{
			forward_back_speed = 30;
			left_right_speed 	 = 30;
		}
		else if(chassis_control->chassis_RC->rc.s[1] == 1)
		{
			forward_back_speed = 30;
			left_right_speed 	 = 30;
		}
		else
		{
			forward_back_speed = 50;
			left_right_speed   = 40;
		}
		
		//W��Sǰ��
		if(chassis_control->chassis_RC->key.v & W)
		{
			chassis_control->vx = forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else if(chassis_control->chassis_RC->key.v & S)
		{
			chassis_control->vx = -forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else
		{
			chassis_control->vx =0;
			ramp_init(&FBSpeedRamp, 200);
		}
		
		//A��Dƽ��
		if(chassis_control->chassis_RC->key.v & A)
		{
			chassis_control->vy = -left_right_speed * ramp_calc(&LRSpeedRamp);
		}
		else if(chassis_control->chassis_RC->key.v & D)
		{
			chassis_control->vy = left_right_speed * ramp_calc(&LRSpeedRamp);
		}
		else
		{
			chassis_control->vy = 0;
			ramp_init(&LRSpeedRamp, 200);
		}
		
		//G ���� PC3
		if(chassis_control->chassis_RC->key.v & G)	
		{
			//��
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
		}
		else
		{
			//����
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
		}
		
		static uint8_t autodengdao = 0;
		static uint32_t nobug, landing_time = 0;
		//QE�ǵ�ģʽ
		if(chassis_control->chassis_RC->rc.s[1] == 3)
		{
			//Q ����ǰ�� PB1
			if((chassis_control->chassis_RC->key.v & Q) && (keyboard_time - last_Press_time>100))	
			{
				last_Press_time = keyboard_time;
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == 1)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);//�ջ�ǰ��
					ofwheel = 0;
				}
				else
				{
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);//����ǰ��
				  ofwheel = 1;
				}
			}
			
			if(ofwheel == 1)
			{
				landing_time = keyboard_time;
			}
			
			//���ִ�������ǰ���ջ�ȥ
			if(chassis_control->tof_h > 20 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == 0)
			{
				ofwheel = 0;
				autodengdao = 1;
				nobug++;
				if(nobug > 20)
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);//�ջ�ǰ��
			}
			else
			{
				nobug = 0;
			}
			
			//E ������� PC2
			if((chassis_control->chassis_RC->key.v & E) && (keyboard_time - last_Press_time>100))	
			{
				last_Press_time = keyboard_time;
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == 1)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);//�������
					obwheel = 0;
				}
				else
				{
					//����
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
					obwheel = 1;
				}
			}
			
			//������
			if(keyboard_time - landing_time < 2500)
			{
				if(chassis_control->gyro_data->pit > 33)
				{	
					//�������
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
					obwheel = 1;
				}
			}
			
			if((chassis_control->tof_h < 9) && autodengdao)
			{
				autodengdao = 0;
				//�������
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
			}
		}	
		
		//Z �ϳ� PC0
		if(chassis_control->chassis_RC->key.v & Z)
		{
			//��
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			tuo_che = 0;
		}
		else
		{
			//����
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			tuo_che = 0;
		}

		//ȡ��ģʽû�������Ǳջ�
		if(chassis_control->chassis_RC->rc.s[1] == 1)
		{
			chassis_control->vw_offset = chassis_control->gyro_data->yaw;
			chassis_control->vw = (chassis_control->vw_offset - chassis_control->gyro_data->yaw) * 2;
		}
		//ǰ�ִ�����û�������Ǳջ�
		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == 1)
		{
			chassis_control->vw_offset = chassis_control->gyro_data->yaw;
			chassis_control->vw = (chassis_control->vw_offset - chassis_control->gyro_data->yaw) * 2;
		}
		else if((chassis_control->chassis_RC->key.v & CTRL) && (keyboard_time - last_Press_time>1000))
		{
			last_Press_time = keyboard_time;
			chassis_control->vw_offset += 180;
			chassis_control->vw = (chassis_control->vw_offset - chassis_control->gyro_data->yaw) * 2;
		}
		else
		{
			chassis_control->vw_offset += chassis_control->chassis_RC->mouse.x * 0.01;
			chassis_control->vw = (chassis_control->vw_offset - chassis_control->gyro_data->yaw) * 2;
		}	
	}
	//ң�ز���������λΪң��ģʽ
	else
	{
		//ǰ��Ϊң���ұ�ͨ��0������Ϊң���ұ�ͨ��1
		if(chassis_control->chassis_RC->rc.s[0] != 2)
		{
			chassis_control->vx =  chassis_control->chassis_RC->rc.ch[1] * 60.0f/660.0f;
			chassis_control->vy =  chassis_control->chassis_RC->rc.ch[0] * 60.0f/660.0f;
		}
		chassis_control->vw_offset += chassis_control->chassis_RC->rc.ch[2] * 30/660 *  0.02;
		chassis_control->vw = (chassis_control->vw_offset - chassis_control->gyro_data->yaw + chassis_control->gyro_angle_start) * 2;
	}
	
	//��ȡ��ǰϵͳʱ��
	chassis_system_time = xTaskGetTickCount();
	
	//�������ϵͳ��ǰʱ���ȥ����ң���жϵ�ǰʱ�䣬˵��û���յ�ң���ź�
	if((chassis_system_time - chassis_control->chassis_RC->time) > 88)
	{
		chassis_control->vx = chassis_control->vy = chassis_control->vw = 0;
	}
	
	//�����̨����
	chassis_control->gimbal_y_offset += chassis_control->chassis_RC->mouse.y * 0.2;
	if(chassis_control->gimbal_y_offset < 1900)
	{
		chassis_control->gimbal_y_offset = 1900;
	}
	else if(chassis_control->gimbal_y_offset > 2620)
	{
		chassis_control->gimbal_y_offset = 2620;
	}
	
	TIM4->CCR1 = chassis_control->gimbal_y_offset;
	
	if(tuo_che == 1)
	{
		chassis_control->vx = -chassis_control->vx;
		chassis_control->vy = -chassis_control->vy;
	}
	
	//�����ٶ��趨
	chassis_control->motor_chassis[0].speed_set = -chassis_control->vx + chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[1].speed_set =  chassis_control->vx + chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[2].speed_set = -chassis_control->vx - chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[3].speed_set =  chassis_control->vx - chassis_control->vy + (int16_t)chassis_control->vw;
	//����PID

	PID_Calc(&chassis_control->motor_speed_pid[0], chassis_control->motor_chassis[0].speed, chassis_control->motor_chassis[0].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[1], chassis_control->motor_chassis[1].speed, chassis_control->motor_chassis[1].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[2], chassis_control->motor_chassis[2].speed, chassis_control->motor_chassis[2].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[3], chassis_control->motor_chassis[3].speed, chassis_control->motor_chassis[3].speed_set);
	
	//��ֵ����ֵ
	chassis_control->motor_chassis[0].give_current = (int16_t)(chassis_control->motor_speed_pid[0].out);
	chassis_control->motor_chassis[1].give_current = (int16_t)(chassis_control->motor_speed_pid[1].out);
	chassis_control->motor_chassis[2].give_current = (int16_t)(chassis_control->motor_speed_pid[2].out);
	chassis_control->motor_chassis[3].give_current = (int16_t)(chassis_control->motor_speed_pid[3].out);
	//printf("%d\r\n",chassis_control->tof_h);
	//Ni_Ming(0xf1,chassis_control->tof_h,0,0,0);
}
