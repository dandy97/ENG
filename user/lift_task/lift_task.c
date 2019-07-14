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
lift_mode_e last_lift_mode = Init_MODE;
pinch_mode_e pinch_mode = PINCH_INIT;
pinch_mode_e last_pinch_mode = PINCH_INIT;

static uint8_t auto_five_state = 0;//你赢了
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

static uint32_t init_time = 0;
//升降数据更新
void lift_feedback_update(lift_move_t *lift_update)
{	
	//初始化过程
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
	//准备过程
	if(lift_mode == Ready_MODE || lift_mode == Reset_MODE)
	{
		if(lift_update->motor_lift[2].angle > 29 && lift_update->motor_lift[2].angle < 40)
		{
			lift_mode = Start_MODE;
		}
	}
	//开始模式
	if(lift_mode != Ready_MODE && lift_mode != Init_MODE && lift_mode != Reset_MODE)
	{
		//更新升降任务状态
		switch(lift_update->lift_RC->rc.s[0])
		{
			case 1:
			{
				if(last_lift_mode == Key_MODE)
				{
					lift_mode = Reset_MODE;//复位模式
				}
				else
				{
					lift_mode = Rc_MODE;//遥控模式
				}
				break;
			}			
			case 3:
			{
				if(last_lift_mode == Rc_MODE)
				{
					lift_mode = Reset_MODE;//复位模式
				}
				else
				{
					lift_mode = Key_MODE;//键盘模式
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
	}
	
	static uint8_t pretect_lift = 0;
	//取弹状态机
	if(lift_update->lift_RC->rc.s[0] == 3)
	{
			if(lift_update->lift_RC->rc.s[1] == 1)
			{
				pretect_lift = 1;
				pinch_mode = PINCH_RISE;
			}
			else if(lift_update->lift_RC->rc.s[1] == 3)
			{
				auto_five_state = 0;//你赢了
				if(pretect_lift == 1)
				{
					lift_mode = Reset_MODE;//复位模式
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
	//遥控信号失去后为停止模式
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//停止模式
	}
	else if(last_lift_mode == Stop_MODE)
	{
		lift_mode = Init_MODE;
	}
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


//升降控制PID计算
void lift_control_loop(lift_move_t *lift_control)
{	
	lift_control->key_time++;
	/**********************************************气缸控制*******************************************************************/
	switch(pinch_mode)
	{
		case PINCH_INIT://初始状态
		{
			HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EXTEND,GPIO_PIN_RESET);
			break;
		}			
		case PINCH_RISE://取弹状态
		{
			
			Set_LIFT_KEY_GPIO(lift_control->lift_RC->key.v, C, EXTEND, lift_control->key_time);//控制伸缩气缸
			contrl_cylinder(lift_control->lift_RC->mouse.press_l, lift_control->lift_RC->mouse.press_r, lift_control->key_time);//鼠标控制取弹
			break;
		}
		case PINCH_GIVE://给弹状态
		{
			
			break;
		}
		default:
		{
			break;
		}
	}
	/**********************************************气缸控制*******************************************************************/
	/**********************************************电机控制*******************************************************************/
	switch(lift_mode)
	{
		case Stop_MODE://停止状态
		{
			lift_control->motor_lift[2].angle_set = lift_control->motor_lift[2].angle;
			lift_control->motor_lift[2].speed_set = 0;
			break;
		}
		case Init_MODE://初始化模式
		{	
			lift_control->motor_lift[0].angle_set = lift_control->motor_lift[0].angle;
			lift_control->motor_lift[2].speed_set = -25;//-25
			break;
		}
		case Ready_MODE://开始模式
		{
			lift_control->motor_lift[2].angle_set = -35.5f;
			break;
		}
		case Reset_MODE://复位模式
		{
			lift_control->motor_lift[2].angle_set = -35.5f;
			break;
		}
		case Rc_MODE://遥控手杆状态
		{
			//升降位置输入
			lift_control->motor_lift[0].angle_set += lift_control->lift_RC->rc.ch[3] * 0.0005f;		
			//lift_control->motor_lift[2].angle_set += lift_control->lift_RC->rc.ch[2] * 0.0005f;//-35
			break;
		}
		case Key_MODE://键盘模式
		{
				switch(lift_control->lift_RC->rc.s[1])
				{
					case 1://初始状态
					{
						//升降位置输入
						lift_control->motor_lift[0].angle_set = 19.0f;
						//平移位置输入
						if(lift_control->motor_lift[0].angle > -10.5f && lift_control->motor_lift[0].angle < -9.0f)
						{
							lift_control->motor_lift[2].angle_set = -35.0f;
						}
						if(lift_control->lift_RC->key.v == Q)//左移
						{
							lift_control->motor_lift[2].angle_set -= 0.1f;
						}
						if(lift_control->lift_RC->key.v == E)//右移
						{
							lift_control->motor_lift[2].angle_set += 0.1f;
						}
						//一次取5个
						Auto_eat_five(lift_control->lift_RC->key.v, F, lift_control->lift_RC->mouse.press_r,lift_control->key_time, lift_control->motor_lift[2].angle, &lift_control->motor_lift[2].angle_set);
						break;
					}
					case 3://升高
					{
						//升降位置输入
						lift_control->motor_lift[0].angle_set = 1.0f;
						break;
					}
					case 2://给弹模式
					{
						//升降位置输入
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
	if(lift_control->motor_lift[2].angle_set < -68.1f)
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
	
	if(lift_mode == Stop_MODE)//停止状态
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
		lift_control->motor_lift[2].give_current = PID_Calc(&lift_control->motor_speed_pid[2], lift_control->motor_lift[2].speed, lift_control->motor_lift[2].speed_set); 
	}
	else if(lift_mode == Init_MODE)//初始状态
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
}

//返回升降状态
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

//void Auto_Pinch()
//{
//	static uint32_t Auto_Pinch_time = 0;
//	Auto_Pinch_time++;
//	if(Auto_Pinch_time)
//	{
//	
//	}
//}	

//取弹按键
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


//鼠标控制气缸
void contrl_cylinder(uint8_t key, uint8_t key1, uint32_t time)
{
	static uint32_t pinch_last_time = 0;
	static int8_t cylinder_state = 0;
	if(key && (time - pinch_last_time >125))//左击
	{
		pinch_last_time = time;
		cylinder_state++;
		if(cylinder_state > 5)
		{
			cylinder_state = 0;
		}
	}
	if(key1 && (time - pinch_last_time >125))//右击
	{
		pinch_last_time = time;
		cylinder_state--;
		if(cylinder_state < 0)
		{
			cylinder_state = 0;
		}
	}
	if(cylinder_state == 0)//初始
	{
		Reset_control;
	}
	if(cylinder_state == 1)//翻
	{
		Bounce_control;
	}
	if(cylinder_state == 2)//翻->夹
	{
		Bounce_Pinch_control;
	}
	if(cylinder_state == 3)//翻->夹->翻
	{
		Bounce_Pinch_Bounce_control;
	}
	if(cylinder_state == 4)//翻->夹->翻->松
	{
		Bounce_Pinch_Bounce_Pinch_control;
	}
	if(cylinder_state == 5)//翻->夹->翻->松->弹开
	{
		Bounce_Pinch_Bounce_Pinch_Flip_control;
	}
}

void Auto_eat_five(uint16_t key, uint16_t key1, uint8_t key2, uint32_t time, float angle, float *a)
{
	static uint32_t auto_last_time, next_time = 0;
	if((key & key1) && (time - auto_last_time > 250))
	{
		auto_last_time = time;
		auto_five_state = 1;
	}
	if(key2 == 1)//鼠标右键退出
	{
		auto_five_state = 0;
	}
	switch(auto_five_state)
	{
		case 0://第一个弹药箱
		{   
			next_time = 0;
			break;
		}
		case 1://第一个弹药箱
		{   
			*a = -35;
			if(angle > 30 && angle < 40)
			{
				next_time++;
				if     (next_time > 200){Bounce_Pinch_Bounce_Pinch_Flip_control;}
				else if(next_time > 150){Bounce_Pinch_Bounce_Pinch_control;}
				else if(Pinch_State){Bounce_Pinch_Bounce_control;}
				else if(next_time > 50) {Bounce_Pinch_control;}
				else if(next_time > 20) {Bounce_control;}
				if(Bounce_State && next_time > 200)
				{
					auto_five_state = 2;
					next_time = 0;
				}
			}
			else
			{
				Reset_control;
			}
			break;
		}
		case 2://第二个弹药箱
		{
			*a = -65;
			if(angle > 60 && angle < 70)
			{
				next_time++;
				if     (next_time > 200){Bounce_Pinch_Bounce_Pinch_Flip_control;}
				else if(next_time > 150){Bounce_Pinch_Bounce_Pinch_control;}
				else if(Pinch_State){Bounce_Pinch_Bounce_control;}
				else if(next_time > 50) {Bounce_Pinch_control;}
				else if(next_time > 20) {Bounce_control;}
				if(Bounce_State && next_time > 200)
				{
					auto_five_state = 3;
					next_time = 0;
				}
			}
			else
			{
				Reset_control;
			}
			break;
		}
		case 3://第三个弹药箱
		{
			*a = -5;
			if(angle > 0 && angle < 10)
			{
				next_time++;
				if     (next_time > 200){Bounce_Pinch_Bounce_Pinch_Flip_control;}
				else if(next_time > 150){Bounce_Pinch_Bounce_Pinch_control;}
				else if(Pinch_State){Bounce_Pinch_Bounce_control;}
				else if(next_time > 50) {Bounce_Pinch_control;}
				else if(next_time > 20) {Bounce_control;}
				if(Bounce_State && next_time > 200)
				{
					auto_five_state = 4;
					next_time = 0;
				}
			}
			else
			{
				Reset_control;
			}
			break;
		}
		case 4://第四个弹药箱
		{
			*a = -25;
			HAL_GPIO_WritePin(EXTEND,GPIO_PIN_SET);
			if(angle > 20 && angle < 30)
			{
				next_time++;
				if     (next_time > 200){Bounce_Pinch_Bounce_Pinch_Flip_control;}
				else if(next_time > 150){Bounce_Pinch_Bounce_Pinch_control;}
				else if(Pinch_State){Bounce_Pinch_Bounce_control;}
				else if(next_time > 50) {Bounce_Pinch_control;}
				else if(next_time > 20) {Bounce_control;}
				if(Bounce_State && next_time > 200)
				{
					auto_five_state = 5;
					next_time = 0;
				}
			}
			break;
		}
		case 5://第五个弹药箱
		{
			HAL_GPIO_WritePin(EXTEND,GPIO_PIN_SET);
			*a = -55;
			if(angle > 50 && angle < 60)
			{
				next_time++;
				if     (next_time > 200){Bounce_Pinch_Bounce_Pinch_Flip_control;}
				else if(next_time > 150){Bounce_Pinch_Bounce_Pinch_control;}
				else if(Pinch_State){Bounce_Pinch_Bounce_control;}
				else if(next_time > 50) {Bounce_Pinch_control;}
				else if(next_time > 20) {Bounce_control;}
				if(Bounce_State && next_time > 200)
				{
					auto_five_state = 6;
					next_time = 0;
				}
			}
			break;
		}
		case 6://第五个弹药箱
		{
			*a = -35;
			HAL_GPIO_WritePin(EXTEND,GPIO_PIN_RESET);
			auto_five_state = 0;
			break;
		}
		default:
		{
			break; 
		}
	}
}
