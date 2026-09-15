/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             锟斤拷锟斤拷锟教拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟教ㄊ癸拷锟斤拷锟斤拷锟斤拷墙锟斤拷锟斤拷锟侥角度ｏ拷锟戒范围锟节ｏ拷-pi,pi锟斤拷
  *             锟绞讹拷锟斤拷锟斤拷目锟斤拷嵌染锟轿拷锟轿э拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆角度硷拷锟斤拷暮锟斤拷锟斤拷锟斤拷锟教拷锟揭拷锟轿?锟斤拷
  *             状态锟斤拷锟斤拷锟斤拷锟角匡拷锟斤拷状态锟斤拷锟斤拷锟矫帮拷锟斤拷锟斤拷锟斤拷锟角斤拷锟斤拷锟斤拷锟教拷墙锟斤拷锌锟斤拷疲锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?  *             状态锟斤拷通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷谋锟斤拷锟街碉拷锟斤拷频锟叫Ｗ硷拷锟斤拷锟斤拷饣癸拷锟叫Ｗ甲刺拷锟酵Ｖ棺刺拷取锟?  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a gimbal behaviour mode
    1. in gimbal_behaviour.h , add a new behaviour name in gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // new add
    }gimbal_behaviour_e,
    2. implement new function. gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" param is gimbal movement contorl input. 
        first param: 'yaw' usually means  yaw axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.
        second param: 'pitch' usually means pitch axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.

        in this new function, you can assign set-point to "yaw" and "pitch",as your wish
    3.  in "gimbal_behavour_set" function, add new logical judgement to assign GIMBAL_XXX_XXX to  "gimbal_behaviour" variable,
        and in the last of the "gimbal_behaviour_mode_set" function, add "else if(gimbal_behaviour == GIMBAL_XXX_XXX)" 
        choose a gimbal control mode.
        four mode:
        GIMBAL_MOTOR_RAW : will use 'yaw' and 'pitch' as motor current set,  derectly sent to can bus.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' are angle increment,  control enconde relative angle.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' are angle increment,  control gyro absolute angle.
    4. in the last of "gimbal_behaviour_control_set" function, add
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }

        
    锟斤拷锟揭拷锟斤拷锟揭伙拷锟斤拷碌锟斤拷锟轿Ｊ?    1.锟斤拷锟饺ｏ拷锟斤拷gimbal_behaviour.h锟侥硷拷锟叫ｏ拷 锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷为锟斤拷锟斤拷锟斤拷 gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // 锟斤拷锟斤拷锟接碉拷
    }gimbal_behaviour_e,

    2. 实锟斤拷一锟斤拷锟铰的猴拷锟斤拷 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 锟斤拷锟斤拷锟斤拷锟斤拷台锟剿讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
        锟斤拷一锟斤拷锟斤拷锟斤拷: 'yaw' 通锟斤拷锟斤拷锟斤拷yaw锟斤拷锟狡讹拷,通锟斤拷锟角角讹拷锟斤拷锟斤拷,锟斤拷值锟斤拷锟斤拷时锟斤拷锟剿讹拷,锟斤拷值锟斤拷顺时锟斤拷
        锟节讹拷锟斤拷锟斤拷锟斤拷: 'pitch' 通锟斤拷锟斤拷锟斤拷pitch锟斤拷锟狡讹拷,通锟斤拷锟角角讹拷锟斤拷锟斤拷,锟斤拷值锟斤拷锟斤拷时锟斤拷锟剿讹拷,锟斤拷值锟斤拷顺时锟斤拷
        锟斤拷锟斤拷锟斤拷碌暮锟斤拷锟? 锟斤拷锟杰革拷 "yaw"锟斤拷"pitch"锟斤拷值锟斤拷要锟侥诧拷锟斤拷
    3.  锟斤拷"gimbal_behavour_set"锟斤拷锟斤拷锟斤拷锟斤拷校锟斤拷锟斤拷锟斤拷碌锟斤拷呒锟斤拷卸希锟斤拷锟絞imbal_behaviour锟斤拷值锟斤拷GIMBAL_XXX_XXX
        锟斤拷gimbal_behaviour_mode_set锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然锟斤拷选锟斤拷一锟斤拷锟斤拷台锟斤拷锟斤拷模式
        3锟斤拷:
        GIMBAL_MOTOR_RAW : 使锟斤拷'yaw' and 'pitch' 锟斤拷为锟斤拷锟斤拷锟斤拷锟斤拷瓒ㄖ?直锟接凤拷锟酵碉拷CAN锟斤拷锟斤拷锟斤拷.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 锟角角讹拷锟斤拷锟斤拷,  锟斤拷锟狡憋拷锟斤拷锟斤拷越嵌锟?
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 锟角角讹拷锟斤拷锟斤拷,  锟斤拷锟斤拷锟斤拷锟斤拷锟角撅拷锟皆角讹拷.
    4.  锟斤拷"gimbal_behaviour_control_set" 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
//#include "detect_task.h"
#include "user_lib.h"
extern gimbal_control_t gimbal_control;
//when gimbal is in calibrating, set buzzer frequency and strenght
//锟斤拷锟斤拷台锟斤拷校准, 锟斤拷锟矫凤拷锟斤拷锟斤拷频锟绞猴拷强锟斤拷
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫断ｏ拷锟斤拷为遥锟斤拷锟斤拷锟侥诧拷锟斤拷锟斤拷锟斤拷位锟斤拷时锟津，诧拷一锟斤拷为0锟斤拷
  * @param          锟斤拷锟斤拷锟揭ｏ拷锟斤拷锟街?  * @param          锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭ｏ拷锟斤拷锟街?  * @param          锟斤拷锟斤拷值
  */
#define rc_deadband_limit(input, output, dealine)        \
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


/**
  * @brief          judge if gimbal reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
/**
  * @brief          通锟斤拷锟叫断斤拷锟劫讹拷锟斤拷锟叫讹拷锟斤拷台锟角否到达极锟斤拷位锟斤拷
  * @param          锟斤拷应锟斤拷慕锟斤拷俣龋锟斤拷锟轿籸ad/s
  * @param          锟斤拷时时锟戒，锟斤拷锟斤拷GIMBAL_CALI_STEP_TIME锟斤拷时锟斤拷锟斤拷锟斤拷
  * @param          锟斤拷录锟侥角讹拷 rad
  * @param          锟斤拷锟斤拷锟侥角讹拷 rad
  * @param          锟斤拷录锟侥憋拷锟斤拷值 raw
  * @param          锟斤拷锟斤拷锟侥憋拷锟斤拷值 raw
  * @param          校准锟侥诧拷锟斤拷 锟斤拷锟揭伙拷锟?锟斤拷一
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷为状态锟斤拷锟斤拷锟斤拷.
  * @param[in]      gimbal_mode_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷锟斤拷台锟斤拷为模式锟斤拷GIMBAL_ZERO_FORCE, 锟斤拷锟斤拷锟斤拷锟斤拷岜伙拷锟斤拷锟?锟斤拷台锟斤拷锟斤拷模式锟斤拷raw模式.原始模式锟斤拷味锟斤拷
  *                 锟借定值锟斤拷直锟接凤拷锟酵碉拷CAN锟斤拷锟斤拷锟斤拷,锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿?.
  * @param[in]      yaw:锟斤拷锟斤拷yaw锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[in]      pitch:锟斤拷锟斤拷pitch锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[in]      gimbal_control_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷始锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷墙嵌瓤锟斤拷疲锟斤拷锟教拷锟教э拷锟絧itch锟结，锟斤拷锟斤拷转yaw锟斤拷
  * @param[out]     yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[out]     pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         锟斤拷锟截匡拷
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台校准锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟絩aw锟斤拷锟狡ｏ拷锟斤拷台锟斤拷抬锟斤拷pitch锟斤拷锟斤拷锟斤拷pitch锟斤拷锟斤拷锟斤拷转yaw锟斤拷锟斤拷锟阶獃aw锟斤拷锟斤拷录锟斤拷时锟侥角度和憋拷锟斤拷值
  * @author         RM
  * @param[out]     yaw:锟斤拷锟斤拷yaw锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[out]     pitch:锟斤拷锟斤拷pitch锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[in]      gimbal_control_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷锟斤拷锟角匡拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷墙嵌瓤锟斤拷疲锟?  * @param[out]     yaw: yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[out]     pitch:pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      gimbal_control_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷锟斤拷值锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟皆角度匡拷锟狡ｏ拷
  * @param[in]      yaw: yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      pitch: pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      gimbal_control_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷疲锟斤拷锟斤拷锟斤拷锟斤拷越嵌瓤锟斤拷疲锟?  * @author         RM
  * @param[in]      yaw: yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      pitch: pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      gimbal_control_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

//锟斤拷台锟斤拷为状态锟斤拷
//static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          the function is called by gimbal_set_mode function in gimbal_task.c
  *                 the function set gimbal_behaviour variable, and set motor mode.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷gimbal_set_mode锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷gimbal_task.c,锟斤拷台锟斤拷为状态锟斤拷锟皆硷拷锟斤拷锟阶刺拷锟斤拷锟斤拷锟?  * @param[out]     gimbal_mode_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //锟斤拷台锟斤拷为状态锟斤拷锟斤拷锟斤拷
    gimbal_behavour_set(gimbal_mode_set);

}

/**
  * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
  *                 accoring to the gimbal_behaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_pitch:pitch axis increment angle,unit rad
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷为锟斤拷锟狡ｏ拷锟斤拷锟捷诧拷同锟斤拷为锟斤拷锟矫诧拷同锟斤拷锟狡猴拷锟斤拷
  * @param[out]     add_yaw:锟斤拷锟矫碉拷yaw锟角讹拷锟斤拷锟斤拷值锟斤拷锟斤拷位 rad
  * @param[out]     add_pitch:锟斤拷锟矫碉拷pitch锟角讹拷锟斤拷锟斤拷值锟斤拷锟斤拷位 rad
  * @param[in]      gimbal_mode_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    
		
		
		
    //锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷锟斤拷锟斤拷 int16_t yaw_channel,pitch_channel
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

		
    rc_add_yaw = (yaw_channel * YAW_RC_SEN - 
		              gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN + 
		             (gimbal_control_set->gimbal_rc_ctrl->key.v & GIMBAL_LEFT_KEY)*0.00008 - 
								 (gimbal_control_set->gimbal_rc_ctrl->key.v & GIMBAL_RIGHT_KEY)*0.00005);
    rc_add_pit = (pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN);

    if (gimbal_control_set->gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          锟斤拷台锟斤拷某些锟斤拷为锟铰ｏ拷锟斤拷要锟斤拷锟教诧拷锟斤拷
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_control.gimbal_behaviour == GIMBAL_INIT || gimbal_control.gimbal_behaviour == GIMBAL_CALI || gimbal_control.gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_control.gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          锟斤拷台锟斤拷某些锟斤拷为锟铰ｏ拷锟斤拷要锟斤拷锟酵Ｖ?  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_control.gimbal_behaviour == GIMBAL_INIT ||gimbal_control.gimbal_behaviour == GIMBAL_CALI || gimbal_control.gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷为状态锟斤拷锟斤拷锟斤拷.
  * @param[in]      gimbal_mode_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    if (gimbal_mode_set->gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //锟斤拷锟斤拷锟斤拷始锟斤拷锟斤拷锟绞憋拷洌拷锟斤拷锟斤拷丫锟斤拷榷锟斤拷锟斤拷锟街狄伙拷锟绞憋拷洌拷顺锟斤拷锟绞硷拷锟阶刺拷锟斤拷卮锟斤拷碌锟斤拷锟斤拷锟斤拷叩锟斤拷锟?        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) )//&& !toe_is_error(DBUS_TOE)
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //锟斤拷锟截匡拷锟斤拷 锟斤拷台状态
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (chassis_spin_watch())
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_SPIN;
    }

//    if( toe_is_error(DBUS_TOE))
//    {
//        gimbal_behaviour = GIMBAL_ZERO_FORCE;
//    }

    //enter init mode
    //锟叫断斤拷锟斤拷init状态锟斤拷


			if (gimbal_mode_set->last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_mode_set->gimbal_behaviour != GIMBAL_ZERO_FORCE && gimbal_mode_set->vision_auto_flag !=1)
			{
					gimbal_mode_set->gimbal_behaviour = GIMBAL_INIT;
			}




}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷锟斤拷台锟斤拷为模式锟斤拷GIMBAL_ZERO_FORCE, 锟斤拷锟斤拷锟斤拷锟斤拷岜伙拷锟斤拷锟?锟斤拷台锟斤拷锟斤拷模式锟斤拷raw模式.原始模式锟斤拷味锟斤拷
  *                 锟借定值锟斤拷直锟接凤拷锟酵碉拷CAN锟斤拷锟斤拷锟斤拷,锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿?.
  * @param[in]      yaw:锟斤拷锟斤拷yaw锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[in]      pitch:锟斤拷锟斤拷pitch锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[in]      gimbal_control_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷始锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷墙嵌瓤锟斤拷疲锟斤拷锟教拷锟教э拷锟絧itch锟结，锟斤拷锟斤拷转yaw锟斤拷
  * @author         RM
  * @param[out]     yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[out]     pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         锟斤拷锟截匡拷
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //锟斤拷始锟斤拷状态锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台校准锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟絩aw锟斤拷锟狡ｏ拷锟斤拷台锟斤拷抬锟斤拷pitch锟斤拷锟斤拷锟斤拷pitch锟斤拷锟斤拷锟斤拷转yaw锟斤拷锟斤拷锟阶獃aw锟斤拷锟斤拷录锟斤拷时锟侥角度和憋拷锟斤拷值
  * @author         RM
  * @param[out]     yaw:锟斤拷锟斤拷yaw锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[out]     pitch:锟斤拷锟斤拷pitch锟斤拷锟斤拷锟皆贾碉拷锟斤拷锟街憋拷锟酵拷锟絚an 锟斤拷锟酵碉拷锟斤拷锟?  * @param[in]      gimbal_control_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //锟叫讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷ｏ拷 锟斤拷锟斤拷录锟斤拷锟斤拷锟叫★拷嵌锟斤拷锟斤拷锟?        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷锟斤拷锟角匡拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷墙嵌瓤锟斤拷疲锟?  * @param[out]     yaw: yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[out]     pitch:pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      gimbal_control_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
		
//    static int16_t yaw_channel = 0, pitch_channel = 0;

//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

//    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
//    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;



    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //锟斤拷锟斤拷锟酵凤拷锟侥匡拷锟街?                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //锟斤拷锟较匡拷锟狡碉拷锟斤拷头锟斤拷目锟斤拷值锟斤拷锟斤拷转锟斤拷锟斤拷装锟斤拷锟斤拷锟?            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TURN_SPEED;
            }
            else
            {
                *yaw -= TURN_SPEED;
            }
        }
        //锟斤拷锟斤拷pi 锟斤拷180锟姐）锟斤拷停止
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷锟斤拷值锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟皆角度匡拷锟狡ｏ拷
  * @param[in]      yaw: yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      pitch: pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      gimbal_control_set: 锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
//    static int16_t yaw_channel = 0, pitch_channel = 0;

//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

//    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
//    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;


}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          锟斤拷台锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷疲锟斤拷锟斤拷锟斤拷锟斤拷越嵌瓤锟斤拷疲锟?  * @author         RM
  * @param[in]      yaw: yaw锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      pitch: pitch锟斤拷嵌瓤锟斤拷疲锟轿拷嵌鹊锟斤拷锟斤拷锟?锟斤拷位 rad
  * @param[in]      gimbal_control_set:锟斤拷台锟斤拷锟斤拷指锟斤拷
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}