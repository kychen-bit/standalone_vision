#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H

#include "can_receive.h"
#include "pid.h"
#include "stm32f4xx.h"
#include "main.h"
#include "user_lib.h"
#include "remote_control.h"
																																																							
#include "user_lib.h"
#define REVOLVER_TASK_INIT_TIME 200

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define PRESS_LONG_TIME 400
#define RevChannel 1

#define  	AN_BULLET       	26218//26215.5802139037//26050.78074866310160427807486631//25900.58021390374 // 26050.78074866310160427807486631 //25650.58021390374//(31458.6962566844)		//单个子弹电机位置增加值  282000     26215.58021390374

#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//弹仓开关标志

#define STIR_SPEED_PID_KP 600.0f//3000  2650 1400 1000
#define STIR_SPEED_PID_KI 0
#define STIR_SPEED_PID_KD 2000
#define STIR_SPEED_PID_MAX_OUT 16384.0f
#define STIR_SPEED_PID_MAX_IOUT 5000.0f

#define STIR_POSITION_PID_KP 0.015f//0.0015f//0.0008f、0.003
#define STIR_POSITION_PID_KI 0.00
#define STIR_POSITION_PID_KD 0.000
#define STIR_POSITION_PID_MAX_OUT 150.0f
#define STIR_POSITION_PID_MAX_IOUT 10.0f								//#define STIR_SPEED_PID_KP 55.0f//3000 
//#define STIR_SPEED_PID_KI 0
//#define STIR_SPEED_PID_KD 4
//#define STIR_SPEED_PID_MAX_OUT 16384.0f
//#define STIR_SPEED_PID_MAX_IOUT 5000.0f

//#define STIR_POSITION_PID_KP 0.5f//0.0008f
//#define STIR_POSITION_PID_KI 0
//#define STIR_POSITION_PID_KD 0.1
//#define STIR_POSITION_PID_MAX_OUT 200.0f
//#define STIR_POSITION_PID_MAX_IOUT 20.0f

#define COVER_SPEED_PID_KP 300.0f
#define COVER_SPEED_PID_KI 0
#define COVER_SPEED_PID_KD 20
#define COVER_SPEED_PID_MAX_OUT 8000.0f
#define COVER_SPEED_PID_MAX_IOUT 5000.0f

#define COVER_POSITION_PID_KP -0.0003f
#define COVER_POSITION_PID_KI 0
#define COVER_POSITION_PID_KD 0
#define COVER_POSITION_PID_MAX_OUT 10.0f
#define COVER_POSITION_PID_MAX_IOUT 10.0f


#define FIRC_SPEED_PID_KP 4.0f
#define FIRC_SPEED_PID_KI 0.25f
#define FIRC_SPEED_PID_KD 0.5f
#define FIRC_SPEED_PID_MAX_OUT 16384.0f
#define FIRC_SPEED_PID_MAX_IOUT 5000.0f

//电机rmp 变化成 旋转速度的比例


typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;

typedef enum
{
	OPEN  = 0,
	CLOSE = 1,
}eCoverState;

#define INIT 0
#define NORMAL 1

typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;

typedef struct
{
	const motor_measure_t *stir_motor_measure;
	pid_type_def stir_motor_speed_pid;         //电机速度PID
	pid_type_def stir_motor_position_pid;         //电机位置PID
	fp32 speed;
	fp32 speed_set;
	fp32 angle;
	fp32 open_angle;
	fp32 close_angle;
	fp32 angle_set;
	fp32 angle_ramp_set;
	fp32 set_ramp_angle;
	fp32 stir_buff_ramp;
	eRevolverCtrlMode Revolver_mode;
	eRevolverCtrlMode Revolver_last_mode;
  int16_t given_current;
	bool_t touch_spot;  //检测触点是否碰上了
	uint8_t touch_time; //触点碰到时间
}Stir_motor_t;

typedef struct
{
  const motor_measure_t *firc3508_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	pid_type_def firc_speed_pid;
} Firc3508_Motor_t;


typedef struct
{
	  const RC_ctrl_t *revolver_rc_ctrl;
	  RC_ctrl_time_t RC_revolver_ctrl_time;
	  Stir_motor_t stir_motor_gun;
	  Stir_motor_t cover_motor;
  	eShootAction actShoot;
	  Firc3508_Motor_t Firc_R;
	  Firc3508_Motor_t Firc_L;
	  eCoverState cover_state;
		ramp_function_source_t  *ramp_revolver;
		first_order_filter_type_t revolver_cmd_slow_set_L;
		first_order_filter_type_t revolver_cmd_slow_set_R;
	  
	  bool_t press_l;
    bool_t last_press_l;
	  bool_t press_r;
    bool_t last_press_r;	
	
		bool_t back_flag;
		bool_t front_flag;
	
	  ramp_function_source_t fric1_ramp_3508;
	  ramp_function_source_t fricr_ramp_3508;
	
		bool_t bullet_less;	
	
	  bool_t mode;
	  bool_t last_mode;

		
		bool_t key_b_state;
		bool_t key_b_last_state;		
		
		int16_t X_state;     
		int16_t X_last_state;
			
		bool_t X_flag;	
		
		int16_t v_fic_set;
} Revolver_Control_t;





fp32 get_Cover_Motor_Angle(void);
extern bool_t get_cover_state(void);

extern bool_t get_shoot_mode(void);
extern void Revolver_task(void const*pvParameters);






#endif
