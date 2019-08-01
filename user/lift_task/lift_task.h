#ifndef LIFT_TASK_H
#define LIFT_TASK_H

#include "rc.h"
#include "can_receive.h"
#include "pid.h"

#define Pai 3.141592653589793f

//上层继电器
/*伸
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

/*
	磁性开关
	弹
	后翻
	前翻
	伸缩
	*/
#define Bounce_State        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
#define Flip_For_State			HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
#define Flip_Back_State    	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)
#define Extend_State        HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)

#define One_Angle						35.5f
#define Two_Angle						69.0f
#define Three_Angle					3.0f
#define Four_Angle					20.5f
#define Five_Angle					55.5f

/*
	气缸控制
	初始
	翻
	翻->夹
	翻->夹->翻
	翻->夹->翻->松
	翻->夹->翻->松->弹开
	*/
 
#define Reset_control    HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
#define Bounce_control   HAL_GPIO_WritePin(FLIP,GPIO_PIN_SET);HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
#define Bounce_Pinch_control HAL_GPIO_WritePin(FLIP,GPIO_PIN_SET);HAL_GPIO_WritePin(PINCH,GPIO_PIN_SET);HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
#define Bounce_Pinch_Bounce_control HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);HAL_GPIO_WritePin(PINCH,GPIO_PIN_SET);HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
#define Bounce_Pinch_Bounce_Pinch_control HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_RESET);
#define Bounce_Pinch_Bounce_Pinch_Flip_control HAL_GPIO_WritePin(FLIP,GPIO_PIN_RESET);HAL_GPIO_WritePin(PINCH,GPIO_PIN_RESET);HAL_GPIO_WritePin(BOUNCE,GPIO_PIN_SET);			

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
	Reset_MODE     = 2,			 //复位模式
	Start_MODE     = 3,      //开始模式
  Rc_MODE			   = 4,      //遥控状态
	Key_MODE  		 = 5,      //键盘状态
	Stop_MODE      = 6,      //停止状态
} lift_mode_e;

typedef enum
{
	Pinch_Init = 0,					//初始状态
	Pinch_Rc,
	Pinch_Key,
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
 	uint8_t pinch;
	uint8_t extend;
	uint8_t flip;
	uint8_t bounce;
} Cylinder_State_t;

typedef struct
{
  const RC_ctrl_t *lift_RC;              //底盘使用的遥控器指针
  Lift_Motor_t motor_lift[3];         	 //底盘电机数据
  PidTypeDef motor_speed_pid[3];         //升降电机速度pid
	PidTypeDef motor_pos_pid[3];           //升降电机位置pid
	Cylinder_State_t cylinder_state;
	lift_mode_e mode;
	
	float lift_left_cail;
	float lift_right_cail;
	float translation_cail;
	
	float key_time;
	float last_key_time;
	uint8_t auto_mode;
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
//鼠标控制气缸
void contrl_cylinder(uint8_t key, uint8_t key1, uint32_t time);
//一次取5个
void Auto_eat_five(uint16_t key, uint16_t key1, uint8_t key2, uint32_t time, float angle, float *a);
//自动取弹
void Auto_Auto_Auto(Cylinder_State_t *cylinde_state, uint8_t *mode);
void Auto_Mvp(Cylinder_State_t *cylinde_state, uint8_t *mode, float *angle, float *angle_set, float *high_angle_set);
#endif
