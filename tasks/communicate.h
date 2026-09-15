#ifndef COMMUNICATE_H
#define COMMUNICATE_H
#include "usb_typdef.h"











extern void communi_task(void const *pvParameters);

void communi_init(void);
void SendDataDebug_s_tuck(SendDataDebug_s* Buf);                       	 	//0X01
void SendDataImu_s_tuck(SendDataImu_s* Buf);                      			//0X02
void SendDataRobotStateInfo_s_tuck(SendDataRobotStateInfo_s* Buf);        	//0X03
void SendDataEvent_s_tuck(SendDataEvent_s* Buf);                     		//0X04
void SendDataPidDebug_s_tuck(SendDataPidDebug_s* Buf);                		//0X05
void SendDataAllRobotHp_s_tuck(SendDataAllRobotHp_s* Buf);                	//0X06
void SendDataGameStatus_s_tuck(SendDataGameStatus_s* Buf);            		//0X07
void SendDataGroundRobotPosition_s_tuck(SendDataGroundRobotPosition_s* Buf);//0X08
void SendDataRobotMotion_s_tuck(SendDataRobotMotion_s* Buf);               	//0X09
void SendDataRfidStatus_s_tuck(SendDataRfidStatus_s* Buf);             		//0X0A
void SendDataRobotStatus_s_tuck(SendDataRobotStatus_s* Buf);               	//0X0B
void SendDataJointState_s_tuck(SendDataJointState_s* Buf);                	//0X0C
void SendDataBuff_s_tuck(SendDataBuff_s* Buf);                          	//0X0D

extern SendDataDebug_s senddatadebug;                         //0X01
extern SendDataImu_s senddataimu;                             //0X02
extern SendDataRobotStateInfo_s senddatarobotstateinfo;       //0X03
extern SendDataEvent_s senddataevent;                         //0X04
extern SendDataPidDebug_s senddatapiddebug;                   //0X05
extern SendDataAllRobotHp_s senddataallrobothp;               //0X06
extern SendDataGameStatus_s senddatagamestatus;               //0X07
extern SendDataGroundRobotPosition_s senddatagroundposition;  //0X08
extern SendDataRobotMotion_s senddatarobotmotion;             //0X09
extern SendDataRfidStatus_s senddatarfidstatus;               //0X0A
extern SendDataRobotStatus_s senddatarobotstatus;             //0X0B
extern SendDataJointState_s senddatajointstate;               //0X0C
extern SendDataBuff_s senddatabuff;                           //0X0D
	























#endif

