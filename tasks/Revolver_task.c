#include "Revolver_task.h"
//#include "calibrate_task.h"
#include "remote_control.h"
//#include "Gimbal_Task.h"
//#include "ramp.h"
//#include "led.h"
#include "cmsis_os.h"
#include "can.h"
#include "sys_config.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "user_lib.h"

Revolver_Control_t Revolver_control;
static void REVOLVER_Init(Revolver_Control_t *revolver_init);
static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback);
static void STIR_UpdateMotorAngleSum(Stir_motor_t *Stir_motor_sum);
bool_t touchspot_bullet_monitor(void);
static void Revolver_stirgun_control_loop(Revolver_Control_t *revolver_control);
static void Firc_Control_loop(Revolver_Control_t *fire_control);

//extern  int16_t yaw_can_set_current;//,Shoot_Can_Set_Current;
extern int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;
void Revolver_task(void const *pvParameters)
{

	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	REVOLVER_Init(&Revolver_control);
	
	for(;;)
	{
		Revolver_Feedback_Update(&Revolver_control); 
		Revolver_stirgun_control_loop(&Revolver_control);
		Firc_Control_loop(&Revolver_control);
		shoot_can_set_current = Revolver_control.stir_motor_gun.given_current;
		CAN_cmd_gimbal(pitch_can_set_current,0,0, 0);
		CAN1_CMD_GIMBAL(0,yaw_can_set_current,0,0);
//		CAN2_CMD_SHOOT(Revolver_control.Firc_L.give_current,Revolver_control.Firc_R.give_current,0,0);
	vTaskDelay(2);
	}
}



static void REVOLVER_Init(Revolver_Control_t *revolver_init)
{
	
	 const static fp32 revolver_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
	 //锟斤拷锟街碉拷锟斤拷锟絧id没锟叫憋拷要使锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷
	 const fp32 Stir_speed_pid[3] = {STIR_SPEED_PID_KP, STIR_SPEED_PID_KI, STIR_SPEED_PID_KD};
	 const fp32 Stir_position_pid[3] = {STIR_POSITION_PID_KP, STIR_POSITION_PID_KI, STIR_POSITION_PID_KD};
	 
	 const fp32 Cover_speed_pid[3] = {COVER_SPEED_PID_KP, COVER_SPEED_PID_KI, COVER_SPEED_PID_KD};
	 const fp32 Cover_position_pid[3] = {COVER_POSITION_PID_KP, COVER_POSITION_PID_KI, COVER_POSITION_PID_KD};
	 
	 const fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};
	 
	 first_order_filter_init(&revolver_init->revolver_cmd_slow_set_L,CHASSIS_CONTROL_TIME_MS,	revolver_order_filter);
	 first_order_filter_init(&revolver_init->revolver_cmd_slow_set_R,CHASSIS_CONTROL_TIME_MS,	revolver_order_filter);
	 //锟斤拷始锟斤拷PID
	 PID_init(&(revolver_init->stir_motor_gun.stir_motor_speed_pid), PID_POSITION, Stir_speed_pid, 
	            STIR_SPEED_PID_MAX_OUT, STIR_SPEED_PID_MAX_IOUT);
	 PID_init(&(revolver_init->stir_motor_gun.stir_motor_position_pid), PID_POSITION, Stir_position_pid, 
	            STIR_POSITION_PID_MAX_OUT, STIR_POSITION_PID_MAX_IOUT);
	 
	 PID_init(&(revolver_init->cover_motor.stir_motor_speed_pid), PID_POSITION, Cover_speed_pid, 
	            COVER_SPEED_PID_MAX_OUT, COVER_SPEED_PID_MAX_IOUT);
	 PID_init(&(revolver_init->cover_motor.stir_motor_position_pid), PID_POSITION, Cover_position_pid, 
	           COVER_POSITION_PID_MAX_OUT, COVER_POSITION_PID_MAX_IOUT);
	 
	 PID_init(&(revolver_init->Firc_R.firc_speed_pid), PID_POSITION, Firc_speed_pid, 
	            FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);
	 PID_init(&(revolver_init->Firc_L.firc_speed_pid), PID_POSITION, Firc_speed_pid, 
	            FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);
	 
   //遥锟斤拷锟斤拷指锟斤拷
   revolver_init->revolver_rc_ctrl=get_remote_control_point();
	 //锟斤拷锟街革拷锟?	 revolver_init->stir_motor_gun.stir_motor_measure = get_trigger_motor_measure_point();
	 revolver_init->Firc_L.firc3508_motor_measure = get_FireL_Motor_Measure_Point();
	 revolver_init->Firc_R.firc3508_motor_measure = get_FireR_Motor_Measure_Point();
	 ramp_init(revolver_init->ramp_revolver,0.5,600000,0);
//	 ramp_init(&revolver_init->fric1_ramp_3508, 1 * 0.001f, 7500, 0);
//   ramp_init(&revolver_init->fricr_ramp_3508, 1 * 0.001f, 7500, 0);
	 revolver_init->actShoot=SHOOT_NORMAL;
	 revolver_init->stir_motor_gun.Revolver_mode=REVOL_SPEED_MODE;
	 revolver_init->stir_motor_gun.stir_buff_ramp =300;
	 
	 revolver_init->cover_state=CLOSE;
	 revolver_init->cover_motor.angle_set = 0;
	 revolver_init->cover_motor.angle =0;
	 revolver_init->cover_motor.open_angle = revolver_init->cover_motor.angle-152000.0f;
	 
	 revolver_init->cover_motor.stir_buff_ramp = 300;
	 Revolver_Feedback_Update(revolver_init);
}


/**
  * @brief          锟斤拷锟斤拷锟斤拷莞锟斤拷锟?  * @author         RM
  * @param[in]      void
  * @retval         void
  */

static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback)
{
//		heat=JUDGE_usGetRemoteHeat_id1_42mm();//锟斤拷锟斤拷锟斤拷锟斤拷
//	  heat_max=JUDGE_usGetHeatLimit_id1_42mm();
//  	heating=(heat_max-heat);
	  /*************锟斤拷锟斤拷锟斤拷锟斤拷瞬锟?************/
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    
    //锟斤拷锟斤拷锟街碉拷锟斤拷俣锟斤拷瞬锟揭伙拷锟?    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    
    //锟斤拷锟阶碉拷通锟剿诧拷
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_feedback->stir_motor_gun.stir_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    revolver_feedback->stir_motor_gun.speed = speed_fliter_3;
    /*************锟斤拷锟斤拷锟斤拷锟斤拷瞬锟?************/
		
		/*************cover锟斤拷锟斤拷瞬锟?************/
    static fp32 speed_fliter_cover_1 = 0.0f;
    static fp32 speed_fliter_cover_2 = 0.0f;
    static fp32 speed_fliter_cover_3 = 0.0f;
    
    //锟斤拷锟斤拷锟街碉拷锟斤拷俣锟斤拷瞬锟揭伙拷锟?    static const fp32 fliter_cover_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    
    //锟斤拷锟阶碉拷通锟剿诧拷
    speed_fliter_cover_1 = speed_fliter_cover_2;
    speed_fliter_cover_2 = speed_fliter_cover_3;
    speed_fliter_cover_3 = speed_fliter_cover_2 * fliter_cover_num[0] + speed_fliter_cover_1 * fliter_cover_num[1] + (revolver_feedback->cover_motor.stir_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_cover_num[2];
    revolver_feedback->cover_motor.speed = speed_fliter_cover_3;
    /*************cover锟斤拷锟斤拷瞬锟?************/
	  
		
		revolver_feedback->Firc_L.speed=revolver_feedback->Firc_L.firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
		revolver_feedback->Firc_R.speed=revolver_feedback->Firc_R.firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
		
//    STIR_UpdateMotorAngleSum(&(revolver_feedback->stir_motor_gun));
		STIR_UpdateMotorAngleSum(&(revolver_feedback->cover_motor));
    revolver_feedback->stir_motor_gun.angle = revolver_feedback->stir_motor_gun.stir_motor_measure->num*-8192 + revolver_feedback->stir_motor_gun.stir_motor_measure->ecd;
    //锟斤拷臧达拷锟?    revolver_feedback->last_press_l = revolver_feedback->press_l;
    revolver_feedback->press_l = revolver_feedback->revolver_rc_ctrl->mouse.press_l;
		
		revolver_feedback->last_press_r = revolver_feedback->press_r;
    revolver_feedback->press_r = IF_KEY_PRESSED_R;
//		
		if(revolver_feedback->cover_state == OPEN)
		revolver_feedback->cover_motor.open_angle=revolver_feedback->cover_motor.angle;
	
		if(revolver_feedback->cover_state == CLOSE)
		revolver_feedback->cover_motor.close_angle=revolver_feedback->cover_motor.angle;	
		
			if(touchspot_bullet_monitor()) //锟斤拷锟姐开锟斤拷
			{
					revolver_feedback->stir_motor_gun.Revolver_mode = REVOL_POSI_MODE;
				if(revolver_feedback->stir_motor_gun.Revolver_last_mode == REVOL_SPEED_MODE && revolver_feedback->stir_motor_gun.Revolver_mode == REVOL_POSI_MODE)
				{
				revolver_feedback->stir_motor_gun.angle_set = revolver_feedback->stir_motor_gun.angle;
				}
//				revolver_feedback->stir_motor_gun.angle=revolver_feedback->stir_motor_gun.angle_set=0;
//				revolver_feedback->stir_motor_gun.Revolver_mode=REVOL_POSI_MODE;
			}
			else
			{
				revolver_feedback->stir_motor_gun.Revolver_mode=REVOL_SPEED_MODE;
			}
					revolver_feedback->stir_motor_gun.Revolver_mode = REVOL_POSI_MODE;

			if(revolver_feedback->stir_motor_gun.stir_motor_measure->speed_rpm< -620 )
				revolver_feedback->bullet_less = 1;
			else
				revolver_feedback->bullet_less = 0;
			
		if(IF_KEY_PRESSED_B)	
		revolver_feedback->key_b_state = 1;
		else
 		revolver_feedback->key_b_state = 0;
		
		

		if(IF_KEY_PRESSED_X)
		 revolver_feedback->X_state = 1;
		else
		 revolver_feedback->X_state = 0;
		if(revolver_feedback->X_state == 1 && revolver_feedback->X_last_state == 0)
		{
			 revolver_feedback->X_flag=!revolver_feedback->X_flag;	
		}
		revolver_feedback->X_last_state = revolver_feedback->X_state;	
		if(switch_is_up(revolver_feedback->revolver_rc_ctrl->rc.s[0]))
			revolver_feedback->X_flag = 1;
}

static void STIR_UpdateMotorAngleSum(Stir_motor_t *Stir_motor_sum)
{		 
	//锟劫斤拷值锟叫断凤拷
	if (abs(Stir_motor_sum->stir_motor_measure->ecd - Stir_motor_sum->stir_motor_measure->last_ecd) > 4096)//转锟斤拷锟斤拷圈
	{		
		//锟斤拷锟轿诧拷锟斤拷锟角讹拷小锟斤拷锟较次诧拷锟斤拷锟角讹拷锟揭癸拷锟剿帮拷圈,锟斤拷说锟斤拷锟斤拷锟轿癸拷锟斤拷锟斤拷锟?		if (Stir_motor_sum->stir_motor_measure->ecd < Stir_motor_sum->stir_motor_measure->last_ecd)//锟斤拷锟斤拷圈锟揭癸拷锟斤拷锟?		{
			//锟窖撅拷转锟斤拷锟斤拷一圈,锟斤拷锟桔硷拷转锟斤拷 8191(一圈) - 锟较达拷 + 锟斤拷锟斤拷
			  Stir_motor_sum->angle += 8191 -  Stir_motor_sum->stir_motor_measure->last_ecd + Stir_motor_sum->stir_motor_measure->ecd;
		}
		else
		{
			//锟斤拷锟斤拷锟斤拷一圈
				Stir_motor_sum->angle -= 8191 - Stir_motor_sum->stir_motor_measure->ecd + Stir_motor_sum->stir_motor_measure->last_ecd;
		}
	}
	else      
	{
		//未锟斤拷锟劫斤拷值,锟桔硷拷锟斤拷转锟斤拷锟侥角度诧拷
			 Stir_motor_sum->angle += Stir_motor_sum->stir_motor_measure->ecd  -  Stir_motor_sum->stir_motor_measure->last_ecd;
	}
	
}


bool_t touchspot_bullet_monitor() 
{
if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == 0)
	return 1;
else 
	return 0;

}


static void Revolver_stirgun_control_loop(Revolver_Control_t *revolver_control)
{
	  static fp32 angle_out;
	  if(switch_is_down(revolver_control->revolver_rc_ctrl->rc.s[0]))
		{
			 revolver_control->stir_motor_gun.given_current = 0;
			 revolver_control->stir_motor_gun.angle_set = revolver_control->stir_motor_gun.angle;
		}
//		else if(switch_is_up(revolver_control->revolver_rc_ctrl->rc.s[0]))
//		{
//			revolver_control->stir_motor_gun.given_current = PID_calc(&revolver_control->stir_motor_gun.stir_motor_speed_pid, revolver_control->stir_motor_gun.speed, -3.0f);
//		}
		else
		{		
					if(switch_is_down(revolver_control->revolver_rc_ctrl->rc.s[1]))
					{
						revolver_control->stir_motor_gun.angle_set = revolver_control->stir_motor_gun.angle;
					}
					if(switch_is_mid(revolver_control->revolver_rc_ctrl->rc.s[1]))
					{
						revolver_control->mode=0;
					}
					else if(switch_is_up(revolver_control->revolver_rc_ctrl->rc.s[1]))
					{
						revolver_control->mode=1;
					}
						if ((revolver_control->last_mode==0&&revolver_control->mode==1)||(revolver_control->press_l == 1&&revolver_control->last_press_l == 0))//)&&get_firc_speed() == 1
					{
//						if( heating>=100 ||heat_max == 0)
            revolver_control->front_flag = 1;
						revolver_control->stir_motor_gun.angle_set+=(float)AN_BULLET;
					}
				

					revolver_control->stir_motor_gun.angle_ramp_set = RAMP_float(revolver_control->stir_motor_gun.angle_set,revolver_control->stir_motor_gun.angle_ramp_set,200);
								
					angle_out = PID_calc(&revolver_control->stir_motor_gun.stir_motor_position_pid, revolver_control->stir_motor_gun.angle, revolver_control->stir_motor_gun.angle_ramp_set,0);
					revolver_control->stir_motor_gun.given_current = PID_calc(&revolver_control->stir_motor_gun.stir_motor_speed_pid, revolver_control->stir_motor_gun.speed, angle_out,0);								
		}
		
			revolver_control->last_mode = revolver_control->mode;
			revolver_control->stir_motor_gun.Revolver_last_mode = revolver_control->stir_motor_gun.Revolver_mode;	 
}


static void Firc_Control_loop(Revolver_Control_t *fire_control)
{
	
	static uint16_t accelerate_time;
	//锟斤拷锟斤拷缺锟斤拷时锟斤拷锟斤拷锟剿?int16_t firce_l_slow,firce_r_slow;
//	if(Robot_status.shooter_id1_42mm_speed_limit==10)
//	fire_control->v_fic_set = 6100;//16m/s  8000
//	if(Robot_status.shooter_id1_42mm_speed_limit==16)
//	fire_control->v_fic_set = 7200;//16m/s  8000
//	else
//	fire_control->v_fic_set = 6100;
	fire_control->v_fic_set = -9100;

	if(fire_control->bullet_less == 0)

	
	if(switch_is_down(fire_control->revolver_rc_ctrl->rc.s[0]))
	{
		fire_control->Firc_L.speed_set=0;
		fire_control->Firc_R.speed_set=0;
		accelerate_time = 0;
	}
	else
	{
		if(switch_is_down(fire_control->revolver_rc_ctrl->rc.s[RevChannel]))
		{
			accelerate_time = 0;
			fire_control->Firc_L.speed_set=0;
			fire_control->Firc_R.speed_set=0;
			Revolver_control.stir_motor_gun.given_current = 0;

//			laser_off();
		}
		else
		{
//			laser_on();
			if(accelerate_time<250)
      accelerate_time++;
			
			switch(accelerate_time/50)
			{
				case 1:  fire_control->Firc_L.speed_set = -fire_control->v_fic_set/5;
				         fire_control->Firc_R.speed_set = fire_control->v_fic_set/5;break;
				
			  case 2:  fire_control->Firc_L.speed_set = -fire_control->v_fic_set/4;
				         fire_control->Firc_R.speed_set = fire_control->v_fic_set/4;break;
				
			  case 3:  fire_control->Firc_L.speed_set = -fire_control->v_fic_set/3;
				         fire_control->Firc_R.speed_set = fire_control->v_fic_set/3;break;
				
				case 4:  fire_control->Firc_L.speed_set = -fire_control->v_fic_set/2;
				         fire_control->Firc_R.speed_set = fire_control->v_fic_set/2;break;
				
				case 5:  fire_control->Firc_L.speed_set = -fire_control->v_fic_set/1;
				         fire_control->Firc_R.speed_set = fire_control->v_fic_set/1;break;
			}
//			 ramp_calc(&fire_control->fric1_ramp_3508, 2000);
//			 ramp_calc(&fire_control->fricr_ramp_3508, 2000);
//			fire_control->Firc_L.speed_set=7500;//-(uint16_t)(fire_control->fric1_ramp_3508.out); //-9000 ;//90000远锟斤拷锟斤拷9200
//			fire_control->Firc_R.speed_set=7500;//(uint16_t)(fire_control->fricr_ramp_3508.out);;//9000;//9800		
		}

	}
		firce_l_slow = PID_calc(&fire_control->Firc_L.firc_speed_pid, fire_control->Firc_L.firc3508_motor_measure->speed_rpm,fire_control->Firc_L.speed_set,0);
		firce_r_slow = PID_calc(&fire_control->Firc_R.firc_speed_pid, fire_control->Firc_R.firc3508_motor_measure->speed_rpm,fire_control->Firc_R.speed_set,0);
	first_order_filter_cali(&fire_control->revolver_cmd_slow_set_L,firce_l_slow);
	first_order_filter_cali(&fire_control->revolver_cmd_slow_set_R,firce_r_slow);
	fire_control->Firc_L.give_current=fire_control->revolver_cmd_slow_set_L.out;
	fire_control->Firc_R.give_current=fire_control->revolver_cmd_slow_set_R.out;
}