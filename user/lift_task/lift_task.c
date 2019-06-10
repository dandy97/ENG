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
lift_mode_t lift_mode = Init_MODE;
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
	//登岛电机初始化
	lift_wheel_init(&lift_wheel);
	while(1)
	{
		//升降数据更新
		lift_feedback_update(&lift_move);
		//升降模式选择
		lift_mode_switch(&lift_move);
		//升降控制PID计算
		lift_control_loop(&lift_move);
		
		lift_wheel_control_loop(&lift_wheel);
		//发送电流值
	  CAN_CMD_LIFT(lift_wheel.motor_chassis[0].give_current, lift_wheel.motor_chassis[1].give_current, lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current);
		//CAN_CMD_LIFT(lift_wheel.motor_chassis[0].give_current, lift_wheel.motor_chassis[1].give_current, 0, 0);
		//Ni_Ming(0xf1,lift_move.motor_lift[1].give_current,0,0,0);
		//CAN_CMD_LIFT(0, 0, -0, 0);
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
	const static float lift_speed_pid[3] = {20, 1, 600};
	//升降位置环PID值
	const static float lift_pos_pid[3] = {8, 0, 0};
	
	//获取遥控指针 
	lift_init->lift_RC = get_remote_control_point();
	
	//获取升降电机数据指针 
	lift_init->motor_lift[0].lift_motor_measure = get_Motor_Measure_Point(6);
	lift_init->motor_lift[1].lift_motor_measure = get_Motor_Measure_Point(7);

	//升降初始状态为静止状态
	lift_init->mode = Lift_Stop;
	
	//初始化升降速度环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 7000, 3000);
	}
	
	//初始化底盘位置环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 500, 0);
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
	
	//更新电机角度
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle;
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle;
	
	//更新升降任务状态
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

//升降模式选择
void lift_mode_switch(lift_move_t *lift_mode)
{
	//当前系统时间和升降任务时间
	static uint32_t lift_task_system_time,life_task_time = 0;
	
	//每4ms累加一
	life_task_time++;
	
	//获取当前系统时间
	lift_task_system_time = xTaskGetTickCount();
	//如果底盘系统当前时间减去进入遥控中断当前时间，说明没有收到遥控信号
	if((lift_task_system_time - lift_mode->lift_RC->time) > 88)
	{
		//停止状态
		lift_mode->mode = Lift_Stop;
	}
	
	//左边拨杆打到最上面，升降状态不是停止状态，升降任务时间大于500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[1] == 1) && (life_task_time > 125))
	{
		//键盘状态
		lift_mode->mode = Lift_Key;
	}
	
	//左边拨杆打到最中下，升降状态不是停止状态，升降任务时间大于500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[1] == 3) && (life_task_time > 125))
	{
		//键盘状态
		lift_mode->mode = Lift_Run;
	}
	
	//左边拨杆打到最中下，升降状态不是停止状态，升降任务时间大于500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[1] == 2) && (life_task_time > 125))
	{
		//键盘状态
		lift_mode->mode = Lift_Give;
	}
	
	//右边拨杆打到中间，升降状态不是停止状态，升降任务时间大于500(125 * 4)ms
	else if((lift_mode->lift_RC->rc.s[0] == 2) && (life_task_time > 125))
	{
		//遥控手杆状态
		lift_mode->mode = Lift_Rc;       
	}

	else
	{
		//停止状态
		lift_mode->mode = Lift_Stop;
	}
}


//取弹状态
static uint8_t pinch_state = 0;
//键盘模式时间 上一次按下按键时间 
static uint32_t lift_keyboard_time, lift_keyboard_last_time, lift_last_Press_time, release_delay, auto_pinch_time= 0;
//取单升降高度
static uint32_t pinch_height = 765;//780
//自动取单标志
static uint8_t auto_pinch = 0;
//升降控制PID计算
void lift_control_loop(lift_move_t *lift_control)
{	
	static uint32_t lift_time_delay = 0;
	switch(lift_control->mode)
	{
		case Lift_Stop://停止状态
		{
			lift_time_delay = 0;
			//升降位置输入
			lift_control->motor_lift[1].angle_set = 100;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			break;
		}
		case Lift_Rc://遥控手杆状态
		{
			lift_time_delay = 0;
			//升降位置输入
			lift_control->motor_lift[1].angle_set += lift_control->lift_RC->rc.ch[3] * 0.002f;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;		
			break;
		}
		case Lift_Run://平时模式
		{
			//1级收 PC5
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
			//松开夹取
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			//不弹弹药箱
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			//2级收
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
			
			//升降位置输入
			lift_control->motor_lift[1].angle_set = 120;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;		
			break;
		}
		case Lift_Give://给弹模式
		{
			lift_control->motor_lift[1].angle_set = 950;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			
			static uint8_t okok = 0;
			static uint32_t okok_delay = 0;
			if(lift_control->motor_lift[1].angle_set > 300)
			{
				//Q 取弹左移 PB0
				if(lift_control->lift_RC->key.v & Q)
				{
					//左移
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				}
				//一键对中
				else if(lift_control->lift_RC->key.v & R)
				{
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 0)//红外对管
					{
						okok = 1;
					}
					
					if(okok == 1)
					{
						//停止左移
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
					}
					else
					{
						//左移
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
					}
				}
				else
				{
					//停止左移
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				}
				
				//E 取弹右移 PA2
				if(lift_control->lift_RC->key.v & E)
				{
					okok_delay = 0;
					okok = 0;
					//右移
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
				}
				else if(okok == 1)
				{
					okok_delay++;
					if(okok_delay < 15)
					{
						//右移
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
					}
					else
					{
						//停止右移
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
					}
				}
				else
				{
					//停止右移
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
				}
			}
			break;
		}
		case Lift_Key://键盘手杆状态
		{
			lift_time_delay = 0;
			//每4ms加一次
			lift_keyboard_time++;
			/********************************************** 升降部分 *******************************************************/
//			//抬高
//			if((lift_control->lift_RC->key.v == Z) && (lift_keyboard_time - lift_last_Press_time>100))
//			{
//				lift_last_Press_time = lift_keyboard_time;
//				lift_control->motor_lift[1].angle_set = 900;
//			}
//			//复位
//			if((lift_control->lift_RC->key.v == X) && (lift_keyboard_time - lift_last_Press_time>100))
//			{
//				lift_last_Press_time = lift_keyboard_time;
//				lift_control->motor_lift[1].angle_set = 50;
//			}
//			
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;	
			
			
			/**************************************************** 夹取部分 ****************************************************/
			//鼠标左键，每400ms一次，状态前进
			release_delay++;
			//R 2级伸缩 PA3
			if((lift_control->lift_RC->key.v & C) && (lift_keyboard_time - lift_last_Press_time>100))	
			{
				lift_last_Press_time = lift_keyboard_time;
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3) == 1)
				{
					//缩
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
					pinch_height = 750;
				}
				else
				{
					//伸
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
					pinch_height = 820 ;
				}
			}
			
			//F 1级自动取弹丸
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
			
			//自动取弹
			if(auto_pinch == 1)
			{
				auto_pinch_time++;
				if(pinch_state == 6)//弹开
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
				else if(pinch_state == 5)//放
				{
					if(release_delay > 188)
					{
						if(auto_pinch_time > 300)
						pinch_state = 6;
						release_delay = 0;
					}
				}		
				else if(pinch_state == 4)//缩
				{
					if(release_delay > 200)
					{
						if(auto_pinch_time > 250)
						pinch_state = 5;
						release_delay = 0;
					}
				}				
				else if(pinch_state == 3)//抬
				{
					if(auto_pinch_time > 200)
					pinch_state = 4;
					release_delay = 0;
				}				
				else if(pinch_state == 2)//夹
				{
					if(auto_pinch_time > 150)
					pinch_state = 3;
				}				
				else if(pinch_state == 1)//伸
				{
					if(auto_pinch_time > 100)
					pinch_state = 2;
				}				
				else if(pinch_state == 0)//原始状态
				{
					release_delay = 0;
					if(auto_pinch_time > 50)
					pinch_state = 1;
				}
				//鼠标左键中止自动取弹
				if((lift_control->lift_RC->mouse.press_l == 1) && (lift_keyboard_time - lift_last_Press_time > 200))
				{
					lift_last_Press_time = lift_keyboard_time;
					pinch_state = 0;
					auto_pinch_time = 0;
					auto_pinch = 0;
				}
			}
			
			//鼠标左键
			else if((lift_control->lift_RC->mouse.press_l == 1) && (lift_keyboard_time - lift_last_Press_time > 50))
			{
				lift_last_Press_time = lift_keyboard_time;
				if(pinch_state == 6)//弹开
				{
					if(release_delay > 50)
					{
						pinch_state = 0;
					}
				}
				else if(pinch_state == 5)//放
				{
					if(release_delay > 188)
					{
						pinch_state = 6;
						release_delay = 0;
					}
				}		
				else if(pinch_state == 4)//缩
				{
					if(release_delay > 200)
					{
						pinch_state = 5;
						release_delay = 0;
					}
				}				
				else if(pinch_state == 3)//抬
				{
					pinch_state = 4;
					release_delay = 0;
				}				
				else if(pinch_state == 2)//夹
				{
					pinch_state = 3;
				}				
				else if(pinch_state == 1)//伸
				{
					pinch_state = 2;
				}				
				else if(pinch_state == 0)//原始状态
				{
					release_delay = 0;
					pinch_state = 1;
				}
			}
			
			//鼠标右键，每400ms一次，状态后退
			else if((lift_control->lift_RC->mouse.press_r == 1) && (lift_keyboard_time - lift_last_Press_time>100))
			{
				lift_last_Press_time = lift_keyboard_time;
				if(pinch_state == 1)//伸
				{
					pinch_state = 0;//原始状态
				}
				else if(pinch_state == 2)//夹
				{
					pinch_state = 1;//伸
				}				
				else if(pinch_state == 3)//抬
				{
					pinch_state = 2;//夹
				}				
				else if(pinch_state == 4)//缩
				{
					pinch_state = 3;//抬
				}			
				else if(pinch_state == 5)//放
				{
					pinch_state = 4;//缩
				}
				else if(pinch_state == 6)//弹开
				{
					pinch_state = 5;//放
				}
			}
			
			//气缸控制
			if(pinch_state == 1)
			{
				//1级伸 PC5
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				//松开夹取 PC1
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//平时高度
				lift_control->motor_lift[1].angle_set = pinch_height;
				//不弹弹药箱 PC4
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 2)
			{
				//1级伸
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				//夹取
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				//平时高度
				lift_control->motor_lift[1].angle_set = pinch_height;
				//不弹弹药箱
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 3)
			{
				//1级伸
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				//夹取
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				//抬高
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//不弹弹药箱
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 4)
			{
				//1级收
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//夹取
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				//抬高
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//不弹弹药箱
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 5)
			{
				//1级收
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//松开夹取
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//抬高
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//不弹弹药箱
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			else if(pinch_state == 6)
			{
				//1级收
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//松开夹取
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//抬高
				lift_control->motor_lift[1].angle_set = pinch_height + 200;
				//弹弹药箱
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
			}
			else
			{
				//1级收
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				//松开夹取
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				//平时高度
				lift_control->motor_lift[1].angle_set = pinch_height;
				//不弹弹药箱
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			
			
			static uint32_t ok_delay = 0;
			static uint8_t okokokok = 0;			
			if(lift_control->motor_lift[1].angle_set > 300)
			{
				//Q 取弹左移 PB0
				if(lift_control->lift_RC->key.v & Q)
				{
					//左移
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				}
				//一键对中
				else if(lift_control->lift_RC->key.v & R)
				{
					if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 0)
					{
						okokokok = 1;
					}
					
					if(okokokok == 1)
					{
						//停止左移
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
					}
					else
					{
						//左移
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
					}
				}
				else
				{
					//停止左移
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				}
				
				//E 取弹右移 PA2
				if(lift_control->lift_RC->key.v & E)
				{
					ok_delay = 0;
					okokokok = 0;
					//右移
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
				}
				else if(okokokok == 1)
				{
					ok_delay++;
					if(ok_delay < 15)
					{
						//右移
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
					}
					else
					{
						//停止右移
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
					}
				}
				else
				{
					//停止右移
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
				}
			}
			break;
		}
		default:
		{
			
		}			
	}
	
	//取弹输入限幅
	if(lift_control->motor_lift[1].angle_set > 1200)
	{
		lift_control->motor_lift[1].angle_set = 1200;
	}
	if(lift_control->motor_lift[1].angle_set < 0)
	{
		lift_control->motor_lift[1].angle_set = 0;
	}
	
	//平时为了提高速度P较大，复位初始位置为了保护结构速度降慢P较小
	if(lift_control->motor_lift[1].angle_set < 200)
	{
		lift_control->motor_pos_pid[0].Kp = lift_control->motor_pos_pid[1].Kp = 3;
	}
	else
	{
		lift_control->motor_pos_pid[0].Kp = lift_control->motor_pos_pid[1].Kp = 8;
	}
	
	//计算PID
	for(uint8_t i = 0; i < 2; i++)
	{
		//位置环
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, lift_control->motor_lift[i].angle_set);
		//速度环
		//复位初始位置为了保护结构i变为0，i积分清零
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
		//速度环PID的i积分输出清0
		lift_control->motor_speed_pid[0].Iout = lift_control->motor_speed_pid[1].Iout = 0;
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
	}
	else
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
		//Ni_Ming(0xf1,lift_control->motor_lift[1].angle,lift_control->motor_lift[0].angle,0,0);
	}
}

