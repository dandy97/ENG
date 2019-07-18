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

// 车身正方向左边升降电机0-ID为7 右边升降电机1-ID为8

//升降任务状态
lift_mode_e lift_mode = Init_MODE;
lift_mode_e last_lift_mode = Init_MODE;
pinch_mode_e pinch_mode = Pinch_Init;
pinch_mode_e last_pinch_mode = Pinch_Init;

//登岛电机数据结构
chassis_move_t lift_wheel;
//升降电机数据结构体
lift_move_t lift_move;
//底盘任务空间剩余量
uint32_t lift_high_water;
//static int32_t pidin, pp, pi, pd, sp, si, sd = 0;
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
		CAN_CMD_LIFT(lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current, lift_move.motor_lift[2].give_current, 0);
		//printf("%d",Flip_State);
		//Ni_Ming(0xf1,lift_move.motor_lift[0].angle ,lift_move.motor_lift[2].angle_set, 0, 0);
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
	
	//升降模式为初始化模式
	lift_mode = Init_MODE;
	//升降速度环PID值
	const static float lift_speed_pid[3] = {400, 0, 180};
	//升降位置环PID值
	const static float lift_pos_pid[3] = {10, 0, 0};
	
	//平移速度位置环
	const static float translation_pos_pid[3] = {9, 0, 0};
	const static float translation_speed_pid[3] = {200, 0, 0};
	
	//获取遥控指针 
	lift_init->lift_RC = get_remote_control_point();
	
	//获取升降电机数据指针 
	lift_init->motor_lift[0].lift_motor_measure = get_Lift_Motor_Measure_Point(0);
	lift_init->motor_lift[1].lift_motor_measure = get_Lift_Motor_Measure_Point(1);
	lift_init->motor_lift[2].lift_motor_measure = get_Lift_Motor_Measure_Point(2);

	//初始化升降速度环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 10000, 3000);
	}
	
	//初始化底盘位置环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 1000, 0);
	}
	
	//初始化平移位置、速度环PID
	PID_Init(&lift_init->motor_pos_pid[2], PID_POSITION, translation_pos_pid, 60, 0);
	PID_Init(&lift_init->motor_speed_pid[2], PID_POSITION, translation_speed_pid, 10000, 3000);
	
	//更新一下数据
  lift_feedback_update(lift_init);
}


//升降数据更新
void lift_feedback_update(lift_move_t *lift_update)
{	
	//初始化过程
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
	//准备过程 平移归中
	if(lift_mode == Ready_MODE || lift_mode == Reset_MODE)
	{
		if(lift_update->motor_lift[2].angle > 29 && lift_update->motor_lift[2].angle < 40)
		{
			lift_mode = Start_MODE;
		}
	}
	//开始模式
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
	
	//遥控信号失去后为停止模式
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//停止模式
		pinch_mode = Pinch_Init;
	}
	else if(last_lift_mode == Stop_MODE)
	{
		lift_mode = Init_MODE;
	}
	
	//更新状态机
	last_lift_mode = lift_mode;//上一次升降状态
	last_pinch_mode = pinch_mode;//上一次取弹状态
	//更新电机速度
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[2].speed = lift_update->motor_lift[2].lift_motor_measure->filter_rate / 19.0f;
	
	//更新电机角度
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->lift_right_cail;//(0~15cm)
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->lift_left_cail;
	lift_update->motor_lift[2].angle = lift_update->motor_lift[2].lift_motor_measure->angle * Pai * 3.0f / 360.0f - lift_update->translation_cail;
}

static uint8_t high_control = 0;
//升降控制PID计算
void lift_control_loop(lift_move_t *lift_control)
{	
	lift_control->key_time++;
	/**********************************************电机控制*******************************************************************/	
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
			if(lift_control->lift_RC->rc.s[0] == 1)//遥控左边最上档位
			{
				high_control = 0;
				if(Extend_State == 1)lift_control->motor_lift[0].angle_set += lift_control->lift_RC->rc.ch[3] * 0.0005;
				lift_control->motor_lift[2].angle_set = -35.5f;
			}
			else if(lift_control->lift_RC->rc.s[0] == 3)//遥控左边中间档位
			{
				if(lift_control->lift_RC->rc.s[1] == 1)//遥控右边最上档位
					{	
						if(high_control == 1)
						{
							lift_control->motor_lift[0].angle_set = 18.0f;
							if(lift_control->motor_lift[0].angle < -16.8f)
							{
								lift_control->cylinder_state.extend = 1;//伸出							
							}
						}
						else if(high_control == 0)
						{
							lift_control->cylinder_state.extend = 0;
							if(Extend_State == 1)
							lift_control->motor_lift[0].angle_set = 14.0f;
						}
						//自动取弹 Q取3个 E取5个 F取一个
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
								high_control = 0;//缩回
							}
						  else if(high_control == 0)
							{
								high_control = 1;//伸出
							}
						}
						//右击取消
						if(lift_control->lift_RC->mouse.press_r == 1)
						{
							lift_control->auto_mode = 0;		
							high_control = 0;
						}
						//自动取弹
						Auto_Mvp(&lift_control->cylinder_state, &lift_control->auto_mode, &lift_control->motor_lift[2].angle, &lift_control->motor_lift[2].angle_set, &lift_control->motor_lift[0].angle_set);
				}
				else if(lift_control->lift_RC->rc.s[1] == 3)//遥控左边中间档位
				{
						high_control = 0;
						if(lift_control->motor_lift[2].angle > 30 && lift_control->motor_lift[2].angle < 40 && Extend_State)lift_control->motor_lift[0].angle_set = 1.0f;
						lift_control->motor_lift[2].angle_set = -35.5f;
				}
				else if(lift_control->lift_RC->rc.s[1] == 2)//遥控左边最下档位
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
	
	//升降高度限幅(2cm ~ 15cm)
	if(lift_control->motor_lift[0].angle_set > 19.0f)
	{
		lift_control->motor_lift[0].angle_set = 19.0;
	}
	else if(lift_control->motor_lift[0].angle_set < 1.01f)
	{
		lift_control->motor_lift[0].angle_set = 1.0;
	}
	
	//平移距离限幅
	if(lift_control->motor_lift[2].angle_set < -69.1f)
	{
		lift_control->motor_lift[2].angle_set = -68.0;
	}
	else if(lift_control->motor_lift[2].angle_set > -4.99f)
	{
		lift_control->motor_lift[2].angle_set = -5.0;
	}
	
	//高度输入
	lift_control->motor_lift[1].angle_set = -lift_control->motor_lift[0].angle_set;
	
	//升降计算PID
	for(uint8_t i = 0; i < 3; i++)
	{
		//位置环
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, -lift_control->motor_lift[i].angle_set);
    //速度环  
		PID_Calc(&lift_control->motor_speed_pid[i], lift_control->motor_lift[i].speed, lift_control->motor_pos_pid[i].out);
	}
	
	if(lift_mode == Stop_MODE || lift_mode == Init_MODE)//停止状态
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
		lift_control->motor_lift[2].give_current = PID_Calc(&lift_control->motor_speed_pid[2], lift_control->motor_lift[2].speed, lift_control->motor_lift[2].speed_set); 
	}
	else if(lift_mode == Ready_MODE)//准备状态
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
		lift_control->motor_lift[2].give_current = lift_control->motor_speed_pid[2].out;
	}
	else//正常状态
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
		lift_control->motor_lift[2].give_current = lift_control->motor_speed_pid[2].out;
	}
	/**********************************************电机控制*******************************************************************/
	

	/**********************************************气缸控制*******************************************************************/
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
	/**********************************************气缸控制*******************************************************************/
}

//返回升降状态
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

static uint8_t auto_mode = 0;
void Auto_Mvp(Cylinder_State_t *cylinde_state, uint8_t *mode, float *angle, float *angle_set, float *high_angle_set)
{
	static uint8_t auto_times, box_times, mmp = 1;
	static uint32_t auto_time = 0;
	if(*mode == 0)//复位
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
	else if(*mode == 1)//取三个
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
	else if(*mode == 2)//取五个
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
		else if(box_times == 4)//第4个
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
		else if(box_times == 5)//第5个
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
	//取一个
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

//自动取弹
void Auto_Auto_Auto(Cylinder_State_t *cylinde_state, uint8_t *mode)
{
	static uint32_t auto_delay_time = 0;
	if(*mode == 1)
	{
		if(auto_mode == 0)//初始状态
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
		else if(auto_mode == 1)//翻出去
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
		else if(auto_mode == 2)//翻出去->夹住
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
		else if(auto_mode == 3)//翻出去->夹住->翻回来
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
		else if(auto_mode == 4)//翻出去->夹住->翻回来->松弹
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
		else if(auto_mode == 5)//复位
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

