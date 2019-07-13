#ifndef LIFT_TASK_H
#define LIFT_TASK_H

#include "rc.h"
#include "can_receive.h"
#include "pid.h"

#define Pai 3.141592653589793f

//上层继电器
/*升
	夹取
	翻转
	弹开
	*/
#define EXTEND				  GPIOE,GPIO_PIN_1
#define PINCH  					GPIOE,GPIO_PIN_2
#define FLIP            GPIOE,GPIO_PIN_3
#define BOUNCE 					GPIOE,GPIO_PIN_4

/*
	限位开关
	*/
#define Limit_Switch    GPIOE,GPIO_PIN_0




/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
#define W 			0x0001		
#define S 			0x0002
#define A 			0x0004
#define D 			0x0008
#define SHIFT 	0x0010
#define CTRL 		0x0020
#define Q 			0x0040
#define E				0x0080
#define R 			0x0100
#define F 			0x0200
#define G 			0x0400
#define Z 			0x0800
#define X 			0x1000
#define C 			0x2000
#define V 			0x4000		
#define B				0x8000
/******************************************************/

typedef enum
{
	Init_MODE      = 0,      //初始状态
	Ready_MODE     = 1,			 //准备模式
	Start_MODE     = 2,      //开始模式
  Rc_MODE			   = 3,      //遥控状态
	Key_MODE  		 = 4,      //键盘状态
	Stop_MODE      = 5,      //停止状态
} lift_mode_e;

typedef enum
{
	PINCH_INIT = 0,					//初始状态
	PINCH_RISE,							//升高
	PINCH_GIVE,							//给弹
}
pinch_mode_e;

typedef struct
{
  const motor_measure_t *lift_motor_measure;
  float speed;
  float speed_set;
	float angle;
	float angle_set;
	float angle_start;
  int16_t give_current;
} Lift_Motor_t;

typedef struct
{
  const RC_ctrl_t *lift_RC;              //底盘使用的遥控器指针
  Lift_Motor_t motor_lift[3];         	 //底盘电机数据
  PidTypeDef motor_speed_pid[3];         //升降电机速度pid
	PidTypeDef motor_pos_pid[3];           //升降电机位置pid
	lift_mode_e mode;
	
	float lift_left_cail;
	float lift_right_cail;
	float translation_cail;
	
	float key_time;
} lift_move_t;

//升降任务
void lift_task(void *pvParameters);
//升降初始化
void lift_init(lift_move_t *lift_init);
//升降数据更新
void lift_feedback_update(lift_move_t *lift_update);
//升降控制PID计算
void lift_control_loop(lift_move_t *lift_control);
//返回取弹状态
uint8_t get_pinch_state(void);
//取弹按键
void Set_LIFT_KEY_GPIO(uint16_t key, uint16_t key1, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t time);
#endif
