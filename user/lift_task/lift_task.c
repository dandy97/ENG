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

// 车身正方向左边升降电机0-ID为7 右边升降电机1-ID为8

//升降任务状态
lift_mode_e lift_mode = Init_MODE;
pinch_mode_e pinch_mode = PINCH_INIT;
//登岛电机数据结构
chassis_move_t lift_wheel;
//升降电机数据结构体
lift_move_t lift_move;
//底盘任务空间剩余量
uint32_t lift_high_water;
void lift_task(void *pvParameters)
{
	//空闲一段时间
  vTaskDelay(2000);
	//升降初始化
	lift_init(&lift_move);
	while(1)
	{
		//升降数据更新
		lift_feedback_update(&lift_move);
		//升降控制PID计算
		lift_control_loop(&lift_move);
		//发送电流值
		CAN_CMD_LIFT(lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current, 0, 0);
		//Ni_Ming(0xf1,-lift_move.motor_lift[0].angle ,lift_move.motor_lift[0].angle_set, lift_move.motor_lift[0].angle_set, 0);
		//控制频率4ms
		vTaskDelay(4);
		lift_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//升降初始化
void lift_init(lift_move_t *lift_init)
{
	if (lift_init == NULL)
	{
		return;
	}
	//升降速度环PID值
	const static float lift_speed_pid[3] = {400, 0, 180};
	//升降位置环PID值
	const static float lift_pos_pid[3] = {10, 0, 0};
	
	//获取遥控指针 
	lift_init->lift_RC = get_remote_control_point();
	
	//获取升降电机数据指针 
	lift_init->motor_lift[0].lift_motor_measure = get_Lift_Motor_Measure_Point(0);
	lift_init->motor_lift[1].lift_motor_measure = get_Lift_Motor_Measure_Point(1);
	lift_init->motor_lift[2].lift_motor_measure = get_Lift_Motor_Measure_Point(2);

	//初始化升降速度环PID 
	for (uint8_t i = 0; i < 3; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 10000, 3000);
	}
	
	//初始化底盘位置环PID 
	for (uint8_t i = 0; i < 3; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 1000, 0);
	}
	
	//更新一下数据
  lift_feedback_update(lift_init);
}

//升降数据更新
void lift_feedback_update(lift_move_t *lift_update)
{
	//更新电机速度
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[2].speed = lift_update->motor_lift[2].lift_motor_measure->filter_rate / 19.0f;
	
	//更新电机角度
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f;//(0~15cm)
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f;
	lift_update->motor_lift[2].angle = lift_update->motor_lift[2].lift_motor_measure->angle;
	
	//更新升降任务状态
	switch(lift_update->lift_RC->rc.s[0])
	{
		case 1:
		{
			lift_mode = Rc_MODE;//遥控模式
			break;
		}			
		case 3:
		{
			lift_mode = Key_MODE;//键盘模式
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
			lift_mode = Stop_MODE;//停止模式
			break;
		}
		default:
		{
			break;
		}
	}
	
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//停止模式
	}
}

//升降控制PID计算
void lift_control_loop(lift_move_t *lift_control)
{	
	switch(lift_mode)
	{
		case Stop_MODE://停止状态
		{
			//升降位置输入
			//lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			break;
		}
		case Rc_MODE://遥控手杆状态
		{
			//升降位置输入
			lift_control->motor_lift[0].angle_set += lift_control->lift_RC->rc.ch[3] * 0.0005f;		
			break;
		}
		case Key_MODE://键盘模式
		{
				switch(pinch_mode)
				{
					case PINCH_INIT://初始状态
					{
						//升降位置输入
						lift_control->motor_lift[0].angle_set = 2.0f;
						break;
					}
					case PINCH_RISE://升高
					{
						//升降位置输入
						lift_control->motor_lift[0].angle_set = 10.0f;
						break;
					}
					case PINCH_GIVE://给弹模式
					{
						//升降位置输入
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
	
	//升降高度限幅(2cm ~ 15cm)
	if(lift_control->motor_lift[0].angle_set > 15.0f)
	{
		lift_control->motor_lift[0].angle_set = 15.0;
	}
	else if(lift_control->motor_lift[0].angle_set < 2.01f)
	{
		lift_control->motor_lift[0].angle_set = 2.0;
	}
	//高度输入
	lift_control->motor_lift[1].angle_set = -lift_control->motor_lift[0].angle_set;
	
	//计算PID
	for(uint8_t i = 0; i < 2; i++)
	{
		//位置环
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, -lift_control->motor_lift[i].angle_set);
    //速度环  
		PID_Calc(&lift_control->motor_speed_pid[i], lift_control->motor_lift[i].speed, lift_control->motor_pos_pid[i].out);
	}
	
	if(lift_mode == Stop_MODE)
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
	}
	else
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
	}
}

//返回升降状态
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

