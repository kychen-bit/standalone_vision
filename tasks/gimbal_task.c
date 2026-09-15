/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"

#include "main.h"
#include "vision.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
//#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
//#include "shoot.h"
#include "pid.h"
#include "usbd_cdc.h"

//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

//#if INCLUDE_uxTaskGetStackHighWaterMark
//uint32_t gimbal_high_water;
//#endif


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */

/**
  * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void GIMBAL_absolute_angle_NOlimit(gimbal_motor_t *gimbal_motor, fp32 add);

static void gimbal_motor_zore_force_control(gimbal_motor_t *gimbal_motor);
	
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor);
	
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      maxout: pid最大输出
  * @param[in]      intergral_limit: pid最大积分输出
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
/**
  * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      get: 角度反馈
  * @param[in]      set: 角度设定
  * @param[in]      error_delta: 角速度
  * @retval         pid 输出
  */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change_double(uint16_t ecd, uint16_t offset_ecd);
//#if GIMBAL_TEST_MODE
////j-scope 帮助pid调参
//static void J_scope_gimbal_test(void);
//#endif
void GIMBAL_AUTO_Mode_Ctrl(gimbal_control_t* gimbal_auto_control);
bool_t rc_vision(gimbal_control_t* Gimbal_Control_rc);
static void Gimbal_kalman_init(gimbal_control_t* gimbal_kalman_init);



//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;
fp32 motor_gyro;  
//motor current 
//发送的电机电流
int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {500, 0, 0, 1000}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {500, 0, 0, 1000}
};//初始化pitch的部分kalman参数

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //云台初始化
    gimbal_init(&gimbal_control);
		TickType_t xLastWakeTime;


    while (1)
    {
				xLastWakeTime = xTaskGetTickCount();
        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
        gimbal_set_control(&gimbal_control);                 //设置云台控制量
        gimbal_control_loop(&gimbal_control);                //云台控制PID计算
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
        CAN_cmd_gimbal(pitch_can_set_current,0,0, 0);//pitch_can_set_current
				CAN1_CMD_GIMBAL(0,yaw_can_set_current,0,0);
				vTaskDelayUntil(&xLastWakeTime,1);
    }
}
				/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
		Gimbal_kalman_init(init);
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
		 static const fp32 Pitch_absolute_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Yaw_absolute_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};
		
		static const fp32 Pitch_relative_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    static const fp32 Yaw_relative_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
		
		
    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
		init->gimbal_behaviour = GIMBAL_ZERO_FORCE;
    //初始化yaw电机pid
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, PID_DELTA, Yaw_relative_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT);
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, PID_DELTA, Yaw_absolute_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
 		PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PID_DELTA, Pitch_relative_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
		PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PID_DELTA, Pitch_absolute_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

		init->gimbal_pitch_motor.max_relative_angle = 0.345;
		init->gimbal_pitch_motor.min_relative_angle = -0.445;
		
		init->gimbal_yaw_motor.max_relative_angle = Motor_Ecd_to_Rad*2048.0f;
		init->gimbal_yaw_motor.min_relative_angle = Motor_Ecd_to_Rad*-2048.0f;
    //清除所有PID

		init->gimbal_yaw_motor.offset_ecd = 5770;
		init->gimbal_pitch_motor.offset_ecd = 7513;
		
    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = 0;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;


    init->gimbal_pitch_motor.absolute_angle_set = 0;//init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = 0;//init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
		



}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
    feedback_update->roll_angle= *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);//
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          780);
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = -*(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
		motor_gyro =feedback_update->gimbal_pitch_motor.motor_gyro;

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
		if(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->num%2 == 1||feedback_update->gimbal_yaw_motor.gimbal_motor_measure->num%2 == -1)
		{
			if(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd<feedback_update->gimbal_yaw_motor.gimbal_motor_measure->last_ecd)
			feedback_update->gimbal_yaw_motor.ecd_angle = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd + 8192;
		else
			feedback_update->gimbal_yaw_motor.ecd_angle = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd + 8192;
		}
		else
			feedback_update->gimbal_yaw_motor.ecd_angle = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change_double(feedback_update->gimbal_yaw_motor.ecd_angle,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
		
		//绝对角度坐标系转换方便电机限位
		feedback_update->gimbal_pitch_motor.trans_angle = feedback_update->gimbal_pitch_motor.absolute_angle - feedback_update->gimbal_pitch_motor.relative_angle;
		feedback_update->gimbal_pitch_motor.max_absoulate_angle = feedback_update->gimbal_pitch_motor.max_relative_angle - feedback_update->gimbal_pitch_motor.trans_angle; //rad
    feedback_update->gimbal_pitch_motor.min_absoulate_angle = feedback_update->gimbal_pitch_motor.min_relative_angle - feedback_update->gimbal_pitch_motor.trans_angle; //rad; //rad
		
		if(rc_vision(feedback_update) == 1)
		{
		feedback_update->vision_flag = 1;
			
    if(feedback_update->vision_flag == 1&&feedback_update->vision_last_flag==0)
       feedback_update->vision_auto_flag = !feedback_update->vision_auto_flag;
		}
		else
		feedback_update->vision_flag = 0;			
    feedback_update->vision_last_flag = feedback_update->vision_flag;
		    if (switch_is_down(feedback_update->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        feedback_update->vision_update_flag = 0;
    }

}








/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change_double(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE*2;
    }
    else if (relative_ecd < -ECD_RANGE)
    {
        relative_ecd += ECD_RANGE*2;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD_DOUBLE;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
		if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_ABSOLUTE_ANGLE&&gimbal_mode_change->gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
		else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_RELATIVE_ANGLE&&gimbal_mode_change->gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
		{
			gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
			gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
		}
		else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_SPIN&&gimbal_mode_change->gimbal_behaviour == GIMBAL_SPIN)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
		else if(gimbal_mode_change->last_gimbal_behaviour != GIMBAL_PARABOLA&&gimbal_mode_change->gimbal_behaviour == GIMBAL_PARABOLA)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
		else if(gimbal_mode_change->last_gimbal_behaviour == GIMBAL_INIT&&gimbal_mode_change->gimbal_behaviour != GIMBAL_INIT)
		{
			gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
			gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
		}
		  gimbal_mode_change->last_gimbal_behaviour = gimbal_mode_change->gimbal_behaviour;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
	if((set_control->gimbal_rc_ctrl->mouse.press_r == 1||set_control->vision_auto_flag == 1)&&set_control->gimbal_behaviour!=GIMBAL_INIT)
		{
			GIMBAL_AUTO_Mode_Ctrl(set_control);
			gimbal_auto_angle_limit(&set_control->gimbal_pitch_motor);
		}
		else
		{
			//遥控器键盘的设置
      gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
		}

    //yaw电机模式控制
    if (set_control->gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        //raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = 0;
			set_control->gimbal_pitch_motor.raw_cmd_current = 0;
    }
    else if (set_control->gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        //gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
				gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);

    }
    else if (set_control->gimbal_behaviour == GIMBAL_SPIN)
    {
				GIMBAL_absolute_angle_NOlimit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
		else if (set_control->gimbal_behaviour == GIMBAL_INIT)
    {
       gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
			 gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
			
    }

}

static void GIMBAL_absolute_angle_NOlimit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //now angle error
    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            //calculate max add_angle
            //计算出一个最大的添加角度，
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_Auto，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor)
{

    if (gimbal_motor == NULL)
    {
        return;
    }
    //now angle error

		
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->absolute_angle  > gimbal_motor->max_absoulate_angle)
    {
				gimbal_motor->absolute_angle_set = gimbal_motor->max_absoulate_angle;
    }
    else if (gimbal_motor->absolute_angle  < gimbal_motor->min_absoulate_angle)
    {
      	gimbal_motor->absolute_angle_set = gimbal_motor->min_absoulate_angle ;
    }
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    if (control_loop->gimbal_behaviour == GIMBAL_INIT)
    {
				gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_motor_zore_force_control(&control_loop->gimbal_yaw_motor);
        gimbal_motor_zore_force_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
				gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_behaviour == GIMBAL_SPIN)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
				gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }

}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set,0);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
		gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set,0);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static void gimbal_motor_zore_force_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = 0;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

void GIMBAL_AUTO_Mode_Ctrl(gimbal_control_t* gimbal_auto_control)
{
	static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref, pitch_angle_ref;//记录目标角度
	
	//获取角度偏差量,欧拉角类型,过分依赖于视觉的精准度
	Vision_Error_Angle_Yaw(&(gimbal_auto_control->gimbal_kalman.Auto_Error_Yaw[NOW])); 
	Vision_Error_Angle_Pitch(&(gimbal_auto_control->gimbal_kalman.Auto_Error_Pitch[NOW]));
	Vision_Get_Distance(&(gimbal_auto_control->gimbal_kalman.Auto_Distance));
	//上面三个函数都是对传入参数的赋值操作，相当于在外部函数提供接口
	
	//对距离进行卡尔曼滤波
	gimbal_auto_control->gimbal_kalman.Auto_Distance = KalmanFilter(&(gimbal_auto_control->gimbal_kalman.Vision_Distance_Kalman), gimbal_auto_control->gimbal_kalman.Auto_Distance);  
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_If_Update() == TRUE)//视觉数据更新了
	{
		//更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
		yaw_angle_ref   = gimbal_auto_control->gimbal_kalman.Auto_Error_Yaw[NOW]   ;//* 30*PI/180.0f;  //1
		pitch_angle_ref = gimbal_auto_control->gimbal_kalman.Auto_Error_Pitch[NOW] ;//* 30*PI/180.0f;  //10 为了将误差放大
		gimbal_auto_control->gimbal_kalman.Vision_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
		gimbal_auto_control->vision_update_flag = 1;
	}
	else if(gimbal_auto_control->vision_update_flag == 0)
	{
		yaw_angle_ref   =  gimbal_auto_control->gimbal_yaw_motor.absolute_angle ;//
		pitch_angle_ref  =  gimbal_auto_control->gimbal_pitch_motor.absolute_angle ;//
	}
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
//	if(gimbal_auto_control->gimbal_kalman.Vision_Time[NOW] != gimbal_auto_control->gimbal_kalman.Vision_Time[LAST])//更新新数据到来的时间
//	{
//		gimbal_auto_control->gimbal_kalman.vision_time_js = gimbal_auto_control->gimbal_kalman.Vision_Time[NOW] - gimbal_auto_control->gimbal_kalman.Vision_Time[LAST];//计算视觉延迟
//		
//		//更新二阶卡尔曼滤波测量值
//		yaw_angle_raw  = yaw_angle_ref;
//		pitch_angle_raw = pitch_angle_ref;        //raw为实际角度 加 视觉传回来的
//		gimbal_auto_control->gimbal_kalman.Vision_Time[LAST] = gimbal_auto_control->gimbal_kalman.Vision_Time[NOW];
//	} //该if只在Vision_If_Update() == TRUE的时候执行
//	
//	//目标速度解算
//	if(VisionRecvData.identify_target == TRUE)//识别到了目标
//	{
//		//卡尔曼滤波速度测量值  Vision_Angle_Speed_Yaw Vision_Angle_Speed_Pitch这两个变量就是通过帧差计算的速度
//		gimbal_auto_control->gimbal_kalman.Vision_Angle_Speed_Yaw = Target_Speed_Calc(&gimbal_auto_control->gimbal_kalman.Vision_Yaw_speed_Struct, gimbal_auto_control->gimbal_kalman.Vision_Time[NOW], yaw_angle_raw);
//		gimbal_auto_control->gimbal_kalman.Vision_Angle_Speed_Pitch = Target_Speed_Calc(&gimbal_auto_control->gimbal_kalman.Vision_Pitch_speed_Struct, gimbal_auto_control->gimbal_kalman.Vision_Time[NOW], pitch_angle_raw); 
//		
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
//		gimbal_auto_control->gimbal_kalman.yaw_kf_result = kalman_filter_calc(&(gimbal_auto_control->gimbal_kalman.yaw_kalman_filter), yaw_angle_raw, gimbal_auto_control->gimbal_kalman.Vision_Angle_Speed_Yaw);
//		gimbal_auto_control->gimbal_kalman.pitch_kf_result = kalman_filter_calc(&(gimbal_auto_control->gimbal_kalman.pitch_kalman_filter), pitch_angle_raw, gimbal_auto_control->gimbal_kalman.Vision_Angle_Speed_Pitch);
//	}
//	else
//	{
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
//		gimbal_auto_control->gimbal_kalman.Vision_Angle_Speed_Yaw = Target_Speed_Calc(&gimbal_auto_control->gimbal_kalman.Vision_Yaw_speed_Struct, xTaskGetTickCount(), gimbal_control.gimbal_yaw_motor.absolute_angle);
//		gimbal_auto_control->gimbal_kalman.Vision_Angle_Speed_Pitch = Target_Speed_Calc(&gimbal_auto_control->gimbal_kalman.Vision_Pitch_speed_Struct, xTaskGetTickCount(),gimbal_control.gimbal_pitch_motor.absolute_angle);
//  	//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
//		gimbal_auto_control->gimbal_kalman.yaw_kf_result = kalman_filter_calc(&(gimbal_auto_control->gimbal_kalman.yaw_kalman_filter), gimbal_control.gimbal_yaw_motor.absolute_angle, 0);
//		gimbal_auto_control->gimbal_kalman.pitch_kf_result = kalman_filter_calc(&(gimbal_auto_control->gimbal_kalman.pitch_kalman_filter), gimbal_control.gimbal_pitch_motor.absolute_angle, 0);
//	}


//	gimbal_control.gimbal_pitch_motor.absolute_angle_set=*gimbal_auto_control->gimbal_kalman.pitch_kf_result;
//	gimbal_control.gimbal_yaw_motor.absolute_angle_set=*gimbal_auto_control->gimbal_kalman.yaw_kf_result;

	gimbal_control.gimbal_pitch_motor.absolute_angle_set=pitch_angle_ref;
	gimbal_control.gimbal_yaw_motor.absolute_angle_set=yaw_angle_ref;
}


bool_t rc_vision(gimbal_control_t* Gimbal_Control_rc)
{
	static int16_t rc_vision_time;
		if (Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[0] > RC_CALI_VALUE_HOLE &&  \
			  Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
		    Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[2] < -RC_CALI_VALUE_HOLE && \
		    Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[3] < -RC_CALI_VALUE_HOLE && \
		    switch_is_down(Gimbal_Control_rc->gimbal_rc_ctrl->rc.s[0]) &&        \
		    switch_is_down(Gimbal_Control_rc->gimbal_rc_ctrl->rc.s[1]) )
		rc_vision_time++;
		else
		rc_vision_time = 0;
		if(rc_vision_time >500)
    return 1;
		else
		return 0;				
}

static void Gimbal_kalman_init(gimbal_control_t* gimbal_kalman_init)
{
	  gimbal_kalman_init->gimbal_kalman.debug_auto_err_p = 40;//45;//35;//14.8;//移动预测系数,越大预测越多
		gimbal_kalman_init->gimbal_kalman.debug_y_sb_sk = 59;//55;
		gimbal_kalman_init->gimbal_kalman.debug_y_sb_brig_sk = 90;//
		gimbal_kalman_init->gimbal_kalman.debug_p_sk = 20;//移动预测系数,越大预测越多
		gimbal_kalman_init->gimbal_kalman.debug_auto_err_y = 120;//角度过大关闭预测
		gimbal_kalman_init->gimbal_kalman.debug_auto_err_p = 150;
		gimbal_kalman_init->gimbal_kalman.debug_kf_delay = 80;//预测延时开启
		gimbal_kalman_init->gimbal_kalman.debug_kf_speed_yl = 0.35;//0.35;//速度过低关闭预测
		gimbal_kalman_init->gimbal_kalman.debug_kf_speed_yl_sb = 0.2;//0.2;//
		gimbal_kalman_init->gimbal_kalman.debug_kf_speed_yh = 5;//速度过高关闭预测
		gimbal_kalman_init->gimbal_kalman.debug_kf_speed_pl = 0.15;//pitch速度过低关闭预测
		gimbal_kalman_init->gimbal_kalman.debug_kf_y_angcon = 220;//125;//115;//135;//预测量限幅
		gimbal_kalman_init->gimbal_kalman.debug_kf_p_angcon = 45;//pitch预测量限幅
		
		//卡尔曼滤波器初始化
		/*PID角度误差卡尔曼,一阶*/
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Pitch_Mech_Error_Kalman), 1, 40);
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Pitch_Gyro_Error_Kalman), 1, 40);
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Yaw_Mech_Error_Kalman), 1, 40);
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Yaw_Gyro_Error_Kalman), 1, 40);
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Vision_Distance_Kalman), 1, 2000);
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Buff_Yaw_Error_Kalman), 1, 0);//底盘打符
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Buff_Pitch_Error_Kalman), 1, 0);//
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Buff_Yaw_Error_Gim_Kalman), 1, 0);//云台打符
		KalmanCreate(&(gimbal_kalman_init->gimbal_kalman.Gimbal_Buff_Pitch_Error_Gim_Kalman), 1, 0);//
	
		/*自瞄卡尔曼滤波,二阶*/
		mat_init(&(gimbal_kalman_init->gimbal_kalman.yaw_kalman_filter.Q),2,2, yaw_kalman_filter_para.Q_data);
		mat_init(&(gimbal_kalman_init->gimbal_kalman.yaw_kalman_filter.R),2,2, yaw_kalman_filter_para.R_data);
		kalman_filter_init(&(gimbal_kalman_init->gimbal_kalman.yaw_kalman_filter), &yaw_kalman_filter_para);
		
		mat_init(&(gimbal_kalman_init->gimbal_kalman.pitch_kalman_filter.Q),2,2, pitch_kalman_filter_para.Q_data);
		mat_init(&(gimbal_kalman_init->gimbal_kalman.pitch_kalman_filter.R),2,2, pitch_kalman_filter_para.R_data);
		kalman_filter_init(&(gimbal_kalman_init->gimbal_kalman.pitch_kalman_filter), &pitch_kalman_filter_para);
}
