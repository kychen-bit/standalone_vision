#include "communicate.h"
#include "gimbal_task.h"
#include "main.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "usbd_cdc.h"
#include "usb_typdef.h"
extern gimbal_control_t gimbal_control;
uint8_t SendData[16];
uint8_t mode;
uint8_t roll_angle[4]={0};
uint8_t pitch_angle[4]={0}; 
uint8_t yaw_angle[4]={0}; 
uint8_t mode;
//发送数据包
SendDataDebug_s senddatadebug;                         //0X01
SendDataImu_s senddataimu;                             //0X02
SendDataRobotStateInfo_s senddatarobotstateinfo;       //0X03
SendDataEvent_s senddataevent;                         //0X04
SendDataPidDebug_s senddatapiddebug;                   //0X05
SendDataAllRobotHp_s senddataallrobothp;               //0X06
SendDataGameStatus_s senddatagamestatus;               //0X07
SendDataGroundRobotPosition_s senddatagroundposition;  //0X08
SendDataRobotMotion_s senddatarobotmotion;             //0X09
SendDataRfidStatus_s senddatarfidstatus;               //0X0A
SendDataRobotStatus_s senddatarobotstatus;             //0X0B
SendDataJointState_s senddatajointstate;               //0X0C
SendDataBuff_s senddatabuff;                           //0X0D

extern USBD_HandleTypeDef hUsbDeviceFS;



void communi_task(void const *pvParameters)
{
	
	vTaskDelay(200);
	communi_init();
    for(;;)
    {
			Float_to_Byte(-gimbal_control.gimbal_pitch_motor.absolute_angle*360/6.28,pitch_angle);//gimbal_control.gimbal_pitch_motor.relative_angle	
			Float_to_Byte(-gimbal_control.gimbal_yaw_motor.absolute_angle*360/6.28,yaw_angle);
			Float_to_Byte(gimbal_control.roll_angle*360/6.28,roll_angle);
			SendDataDebug_s_tuck(&senddatadebug);   
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatadebug,sizeof(SendDataDebug_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);	
			osDelay(1);
		    SendDataImu_s_tuck(&senddataimu);
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddataimu,sizeof(SendDataImu_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataRobotStateInfo_s_tuck(&senddatarobotstateinfo); 
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatarobotstateinfo,sizeof(SendDataRobotStateInfo_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataEvent_s_tuck(&senddataevent); 
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddataevent,sizeof(SendDataEvent_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataPidDebug_s_tuck(&senddatapiddebug);
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatapiddebug,sizeof(SendDataPidDebug_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataAllRobotHp_s_tuck(&senddataallrobothp); 
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddataallrobothp,sizeof(SendDataAllRobotHp_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataGameStatus_s_tuck(&senddatagamestatus); 
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatagamestatus,sizeof(SendDataGameStatus_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataGroundRobotPosition_s_tuck(&senddatagroundposition);
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatagroundposition,sizeof(SendDataGroundRobotPosition_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataRobotMotion_s_tuck(&senddatarobotmotion);
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatarobotmotion,sizeof(SendDataGroundRobotPosition_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataRfidStatus_s_tuck(&senddatarfidstatus);
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatarfidstatus,sizeof(SendDataRobotMotion_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataRobotStatus_s_tuck(&senddatarobotstatus); 
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatarobotstatus,sizeof(SendDataRobotStatus_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataJointState_s_tuck(&senddatajointstate);
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatajointstate,sizeof(SendDataJointState_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
		    SendDataBuff_s_tuck(&senddatabuff); 		
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS,(uint8_t*)&senddatabuff,sizeof(SendDataBuff_s));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
			osDelay(1);
	}
	
}

void communi_init(){
	memset(&senddatadebug,0,sizeof(SendDataDebug_s));                    		//0X01
	memset(&senddataimu,0,sizeof(SendDataImu_s));                        		//0X02
	memset(&senddatarobotstateinfo,0,sizeof(SendDataRobotStateInfo_s));         //0X03
	memset(&senddataevent,0,sizeof(SendDataEvent_s));               			//0X04
	memset(&senddatapiddebug,0,sizeof(SendDataPidDebug_s));            			//0X05
	memset(&senddataallrobothp,0,sizeof(SendDataAllRobotHp_s));          		//0X06
	memset(&senddatagamestatus,0,sizeof(SendDataGameStatus_s));				 	//0X07
	memset(&senddatagroundposition,0,sizeof(SendDataGroundRobotPosition_s));    //0X08
	memset(&senddatarobotmotion,0,sizeof(SendDataRobotMotion_s));       		//0X09
	memset(&senddatarfidstatus,0,sizeof(SendDataRfidStatus_s));          		//0X0A
	memset(&senddatarobotstatus,0,sizeof(SendDataRobotStatus_s));       		//0X0B
	memset(&senddatajointstate,0,sizeof(SendDataJointState_s));         		//0X0C
	memset(&senddatabuff,0,sizeof(SendDataBuff_s));                     	 	//0X0D
	
	senddatadebug.frame_header.sof=0X5A;
	senddatadebug.frame_header.len=sizeof(SendDataDebug_s);
	senddatadebug.frame_header.id=0X01;
	senddatadebug.frame_header.crc=0X00;
	senddatadebug.checksum=0X00;
	
	senddataimu.frame_header.sof=0X5A;
	senddataimu.frame_header.len=sizeof(SendDataDebug_s);
	senddataimu.frame_header.id=0X02;
	senddataimu.frame_header.crc=0X00;
	senddataimu.crc=0X00;
	
	senddatarobotstateinfo.frame_header.sof=0X5A;
	senddatarobotstateinfo.frame_header.len=sizeof(SendDataDebug_s);
	senddatarobotstateinfo.frame_header.id=0X03;
	senddatarobotstateinfo.frame_header.crc=0X00;
	senddatarobotstateinfo.crc=0X00;
	
	senddataevent.frame_header.sof=0X5A;
	senddataevent.frame_header.len=sizeof(SendDataDebug_s);
	senddataevent.frame_header.id=0X04;
	senddataevent.frame_header.crc=0X00;
	senddataevent.crc=0X00;
	
	senddatapiddebug.frame_header.sof=0X5A;
	senddatapiddebug.frame_header.len=sizeof(SendDataDebug_s);
	senddatapiddebug.frame_header.id=0X05;
	senddatapiddebug.frame_header.crc=0X00;
	senddatapiddebug.crc=0X00;
	
	senddataallrobothp.frame_header.sof=0X5A;
	senddataallrobothp.frame_header.len=sizeof(SendDataDebug_s);
	senddataallrobothp.frame_header.id=0X06;
	senddataallrobothp.frame_header.crc=0X00;
	senddataallrobothp.crc=0X00;
	
	senddatagamestatus.frame_header.sof=0X5A;
	senddatagamestatus.frame_header.len=sizeof(SendDataDebug_s);
	senddatagamestatus.frame_header.id=0X07;
	senddatagamestatus.frame_header.crc=0X00;
	senddatagamestatus.crc=0X00;
	
	senddatagroundposition.frame_header.sof=0X5A;
	senddatagroundposition.frame_header.len=sizeof(SendDataDebug_s);
	senddatagroundposition.frame_header.id=0X08;
	senddatagroundposition.frame_header.crc=0X00;
	senddatagroundposition.crc=0X00;
	
	senddatarobotmotion.frame_header.sof=0X5A;
	senddatarobotmotion.frame_header.len=sizeof(SendDataDebug_s);
	senddatarobotmotion.frame_header.id=0X09;
	senddatarobotmotion.frame_header.crc=0X00;
	senddatarobotmotion.crc=0X00;
	
	senddatarfidstatus.frame_header.sof=0X5A;
	senddatarfidstatus.frame_header.len=sizeof(SendDataDebug_s);
	senddatarfidstatus.frame_header.id=0X0A;
	senddatarfidstatus.frame_header.crc=0X00;
	senddatarfidstatus.crc=0X00;
	
	senddatarobotstatus.frame_header.sof=0X5A;
	senddatarobotstatus.frame_header.len=sizeof(SendDataDebug_s);
	senddatarobotstatus.frame_header.id=0X0B;
	senddatarobotstatus.frame_header.crc=0X00;
	senddatarobotstatus.crc=0X00;
	
	senddatajointstate.frame_header.sof=0X5A;
	senddatajointstate.frame_header.len=sizeof(SendDataDebug_s);
	senddatajointstate.frame_header.id=0X0C;
	senddatajointstate.frame_header.crc=0X00;
	senddatajointstate.crc=0X00;
	
	senddatabuff.frame_header.sof=0X5A;
	senddatabuff.frame_header.len=sizeof(SendDataDebug_s);
	senddatabuff.frame_header.id=0X0D;
	senddatabuff.frame_header.crc=0X00;
	senddatabuff.crc=0X00;	
}

void SendDataDebug_s_tuck(SendDataDebug_s* Buf){
	Buf->time_stamp=0;

}	//0X01
void SendDataImu_s_tuck(SendDataImu_s* Buf){
	Buf->time_stamp=0;
	Buf->data.yaw[0]=0;
	Buf->data.yaw[1]=0;
	Buf->data.yaw[2]=0;
	Buf->data.yaw[3]=0;
	Buf->data.pitch[0]=0;
	Buf->data.pitch[1]=0;
	Buf->data.pitch[2]=0;
	Buf->data.pitch[3]=0;
	Buf->data.roll[0]=0;
	Buf->data.roll[1]=0;
	Buf->data.roll[2]=0;
	Buf->data.roll[3]=0;
	Buf->data.yaw_vel[0]=0;
	Buf->data.yaw_vel[1]=0;
	Buf->data.yaw_vel[2]=0;
	Buf->data.yaw_vel[3]=0;
	Buf->data.pitch_vel[0]=0;
	Buf->data.pitch_vel[1]=0;
	Buf->data.pitch_vel[2]=0;
	Buf->data.pitch_vel[3]=0;
	Buf->data.roll_vel[0]=0;
	Buf->data.roll_vel[1]=0;
	Buf->data.roll_vel[2]=0;
	Buf->data.roll_vel[3]=0;

}	//0X02
void SendDataRobotStateInfo_s_tuck(SendDataRobotStateInfo_s* Buf){
	Buf->time_stamp=0;
	Buf->data.type.chassis=0;
	Buf->data.type.gimbal=0;
	Buf->data.type.shoot=0;
	Buf->data.type.arm=0;
	Buf->data.type.custom_controller=0;
	Buf->data.type.reserve=0;
	Buf->data.state.chassis=0;
	Buf->data.state.gimbal=0;
	Buf->data.state.shoot=0;
	Buf->data.state.arm=0;
	Buf->data.state.custom_controller=0;
	Buf->data.state.reserve=0;

}	//0X03
void SendDataEvent_s_tuck(SendDataEvent_s* Buf){
	Buf->time_stamp=0;
	Buf->data.non_overlapping_supply_zone=0;
	Buf->data.overlapping_supply_zone=0;
	Buf->data.supply_zone=0;
	Buf->data.small_energy=0;
	Buf->data.big_energy=0;
	Buf->data.central_highland=0;
	Buf->data.reserved1=0;
	Buf->data.trapezoidal_highland=0;
	Buf->data.center_gain_zone=0;
	Buf->data.reserved2=0;
}	//0X04
void SendDataPidDebug_s_tuck(SendDataPidDebug_s* Buf){
	Buf->time_stamp=0;
	Buf->data.fdb[0]=0;
	Buf->data.fdb[1]=0;
	Buf->data.fdb[2]=0;
	Buf->data.fdb[3]=0;
	Buf->data.ref[0]=0;
	Buf->data.ref[1]=0;
	Buf->data.ref[2]=0;
	Buf->data.ref[3]=0;
	Buf->data.pid_out[0]=0;
	Buf->data.pid_out[1]=0;
	Buf->data.pid_out[2]=0;
	Buf->data.pid_out[3]=0;
}	//0X05

// 0X06 全场机器人hp信息数据包初始化函数
void SendDataAllRobotHp_s_tuck(SendDataAllRobotHp_s* Buf){
    Buf->time_stamp=0;
    Buf->data.red_1_robot_hp=0;
    Buf->data.red_2_robot_hp=0;
    Buf->data.red_3_robot_hp=0;
    Buf->data.red_4_robot_hp=0;
    Buf->data.red_7_robot_hp=0;
    Buf->data.red_outpost_hp=0;
    Buf->data.red_base_hp=0;
    Buf->data.blue_1_robot_hp=0;
    Buf->data.blue_2_robot_hp=0;
    Buf->data.blue_3_robot_hp=0;
    Buf->data.blue_4_robot_hp=0;
    Buf->data.blue_7_robot_hp=0;
    Buf->data.blue_outpost_hp=0;
    Buf->data.blue_base_hp=0;
}	//0X06

// 0X07 比赛信息数据包初始化函数
void SendDataGameStatus_s_tuck(SendDataGameStatus_s* Buf){
    Buf->time_stamp=0;
    Buf->data.game_progress=0;
    Buf->data.stage_remain_time=0;
}	//0X07

// 0X08 地面机器人位置数据包初始化函数
void SendDataGroundRobotPosition_s_tuck(SendDataGroundRobotPosition_s* Buf){
    Buf->time_stamp=0;
    Buf->data.hero_x[0]=0;
    Buf->data.hero_x[1]=0;
    Buf->data.hero_x[2]=0;
    Buf->data.hero_x[3]=0;
    Buf->data.hero_y[0]=0;
    Buf->data.hero_y[1]=0;
    Buf->data.hero_y[2]=0;
    Buf->data.hero_y[3]=0;
    Buf->data.engineer_x[0]=0;
    Buf->data.engineer_x[1]=0;
    Buf->data.engineer_x[2]=0;
    Buf->data.engineer_x[3]=0;
    Buf->data.engineer_y[0]=0;
    Buf->data.engineer_y[1]=0;
    Buf->data.engineer_y[2]=0;
    Buf->data.engineer_y[3]=0;
    Buf->data.standard_3_x[0]=0;
    Buf->data.standard_3_x[1]=0;
    Buf->data.standard_3_x[2]=0;
    Buf->data.standard_3_x[3]=0;
    Buf->data.standard_3_y[0]=0;
    Buf->data.standard_3_y[1]=0;
    Buf->data.standard_3_y[2]=0;
    Buf->data.standard_3_y[3]=0;
    Buf->data.standard_4_x[0]=0;
    Buf->data.standard_4_x[1]=0;
    Buf->data.standard_4_x[2]=0;
    Buf->data.standard_4_x[3]=0;
    Buf->data.standard_4_y[0]=0;
    Buf->data.standard_4_y[1]=0;
    Buf->data.standard_4_y[2]=0;
    Buf->data.standard_4_y[3]=0;
    Buf->data.standard_5_x[0]=0;
    Buf->data.standard_5_x[1]=0;
    Buf->data.standard_5_x[2]=0;
    Buf->data.standard_5_x[3]=0;
    Buf->data.standard_5_y[0]=0;
    Buf->data.standard_5_y[1]=0;
    Buf->data.standard_5_y[2]=0;
    Buf->data.standard_5_y[3]=0;
}    //0X08

// 0X09 机器人运动数据包初始化函数
void SendDataRobotMotion_s_tuck(SendDataRobotMotion_s* Buf){
    Buf->time_stamp=0;
    Buf->data.speed_vector.vx[0]=0;
    Buf->data.speed_vector.vx[1]=0;
    Buf->data.speed_vector.vx[2]=0;
    Buf->data.speed_vector.vx[3]=0;
    Buf->data.speed_vector.vy[0]=0;
    Buf->data.speed_vector.vy[1]=0;
    Buf->data.speed_vector.vy[2]=0;
    Buf->data.speed_vector.vy[3]=0;
    Buf->data.speed_vector.wz[0]=0;
    Buf->data.speed_vector.wz[1]=0;
    Buf->data.speed_vector.wz[2]=0;
    Buf->data.speed_vector.wz[3]=0;
}	//0X09

// 0X0A RFID状态数据包初始化函数
void SendDataRfidStatus_s_tuck(SendDataRfidStatus_s* Buf){
    Buf->time_stamp=0;
    Buf->data.base_gain_point=0;
    Buf->data.central_highland_gain_point=0;
    Buf->data.enemy_central_highland_gain_point=0;
    Buf->data.friendly_trapezoidal_highland_gain_point=0;
    Buf->data.enemy_trapezoidal_highland_gain_point=0;
    Buf->data.friendly_fly_ramp_front_gain_point=0;
    Buf->data.friendly_fly_ramp_back_gain_point=0;
    Buf->data.enemy_fly_ramp_front_gain_point=0;
    Buf->data.enemy_fly_ramp_back_gain_point=0;
    Buf->data.friendly_central_highland_lower_gain_point=0;
    Buf->data.friendly_central_highland_upper_gain_point=0;
    Buf->data.enemy_central_highland_lower_gain_point=0;
    Buf->data.enemy_central_highland_upper_gain_point=0;
    Buf->data.friendly_highway_lower_gain_point=0;
    Buf->data.friendly_highway_upper_gain_point=0;
    Buf->data.enemy_highway_lower_gain_point=0;
    Buf->data.enemy_highway_upper_gain_point=0;
    Buf->data.friendly_fortress_gain_point=0;
    Buf->data.friendly_outpost_gain_point=0;
    Buf->data.friendly_supply_zone_non_exchange=0;
    Buf->data.friendly_supply_zone_exchange=0;
    Buf->data.friendly_big_resource_island=0;
    Buf->data.enemy_big_resource_island=0;
    Buf->data.center_gain_point=0;
    Buf->data.reserved=0;
}	//0X0A

// 0X0B 机器人状态数据包初始化函数（修正：参数改为指针，否则修改无效）
void SendDataRobotStatus_s_tuck(SendDataRobotStatus_s* Buf){
    Buf->time_stamp=0;
    Buf->data.robot_id=0;
    Buf->data.robot_level=0;
    Buf->data.current_up=0;
    Buf->data.maximum_hp=0;
    Buf->data.shooter_barrel_cooling_value=0;
    Buf->data.shooter_barrel_heat_limit=0;
    Buf->data.shooter_17mm_1_barrel_heat=0;
    Buf->data.robot_pos_x[0]=0;
    Buf->data.robot_pos_x[1]=0;
    Buf->data.robot_pos_x[2]=0;
    Buf->data.robot_pos_x[3]=0;
    Buf->data.robot_pos_y[0]=0;
    Buf->data.robot_pos_y[1]=0;
    Buf->data.robot_pos_y[2]=0;
    Buf->data.robot_pos_y[3]=0;
    Buf->data.robot_pos_angle[0]=0;
    Buf->data.robot_pos_angle[1]=0;
    Buf->data.robot_pos_angle[2]=0;
    Buf->data.robot_pos_angle[3]=0;
    Buf->data.armor_id=0;
    Buf->data.hp_deduction_reason=0;
    Buf->data.projectile_allowance_17mm=0;
    Buf->data.remaining_gold_coin=0;
}	//0X0B

// 0X0C 云台状态数据包初始化函数
void SendDataJointState_s_tuck(SendDataJointState_s* Buf){
    Buf->time_stamp=0;
    Buf->data.pitch[0]=0;
    Buf->data.pitch[1]=0;
    Buf->data.pitch[2]=0;
    Buf->data.pitch[3]=0;
    Buf->data.yaw[0]=0;
    Buf->data.yaw[1]=0;
    Buf->data.yaw[2]=0;
    Buf->data.yaw[3]=0;
}	//0X0C

// 0X0D 机器人增益和底盘能量数据包初始化函数
void SendDataBuff_s_tuck(SendDataBuff_s* Buf){
    Buf->time_stamp=0;
    Buf->data.recovery_buff=0;
    Buf->data.cooling_buff=0;
    Buf->data.defence_buff=0;
    Buf->data.vulnerability_buff=0;
    Buf->data.attack_buff=0;
    Buf->data.remaining_energy=0;
}	//0X0D


