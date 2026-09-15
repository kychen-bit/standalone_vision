/**
  **********************************2022 YSU***********************************
  * @file    referee_usart_task.c
  * @brief   裁判系统通讯任务
  ******************************************************************************
	* @team    燕鹰
	* @author  张兆荣
  ******************************************************************************
  * @attention
  * 
  * 此任务用于和裁判系统交换数据，包括操作手UI界面的绘制。
  *
  **********************************2022 YSU***********************************
  */
#include "chassis_task.h"
#include "referee_usart_task.h"
#include "usart.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "gimbal_task.h"
#include "user_lib.h"
#include "main.h"
#include "cmsis_os.h"
//#include "Chassis_Leg_Task.h"
/* Private define ------------------------------------------------------------*/
#define Max(a,b) ((a) > (b) ? (a) : (b))
#define Robot_ID_Current Robot_ID_Red_Sentry
#define l5 0.11f
#define R 60

/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;

/* 动态UI数据变量 */
uint8_t UI_AutoAim_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
float		UI_Chassis_Angle= 0.0f;
uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启

/* 中央标尺高度变量 */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;

uint8_t autoaim_mode;//2:normal,3:small energy,4:big energy
uint8_t autoaim_armor;//0x10:auto,0x20:big,0x30:small
uint8_t if_predict;

bool_t follow_mode=0;
bool_t raw_mode=0;

void ui_update_data(void)
{
	UI_Chassis_Angle=0;
//	-motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offsetEcd);

}
uint8_t ID_Change(uint8_t id){
	if(id<=9)return id;
	switch(id){
		case 103:
			return 103;
		case 104:
			return 104;
		case 105:
			return 105;
		case 106:
			return 106;	
		case 107:
			return 107;			
	}
	return 0;
}

void referee_usart_task(void const * argument)
{

		/* 动态UI控制变量 */
	uint16_t UI_PushUp_Counter = 261;
	
	/* 裁判系统初始化 */
	  vTaskDelay(300);
//		Sentinel_decisions.decisions.sentry_cmd=0;
//    Sentinel_decisions_change(&Sentinel_decisions.decisions);
	/* new UI */
	while(1)
	{
		/* 解析裁判系统数据 */
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
//		Sentinel_decisions_PushUp(&Sentinel_decisions,ID_Change(Game_Robot_State.robot_id));
		vTaskDelay(10);



		/* UI更新 */
		if(0){
		ui_update_data();//更新ui动态数据
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 301 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //第二行右横线
			UI_PushUp_Graphs(7, &UI_Graph7, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		if(UI_PushUp_Counter % 311 == 0) //静态UI预绘制 中央标尺2
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
//			UI_PushUp_Graphs(7, &UI_Graph7, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		if(UI_PushUp_Counter % 321 == 0) //静态UI预绘制 小陀螺预警线
		{
			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  270,   0,  490,  380);//左车体边界线
			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  1430,  380,  1650,  0);//车体右边界线
//			UI_PushUp_Graphs(2, &UI_Graph5, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		if(UI_PushUp_Counter % 331 == 0) //动态UI预绘制 图形
		{
//			UI_Draw_Float (&UI_Graph5.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith轴角度
			UI_Draw_Circle(&UI_Graph5.Graphic[1], "204", UI_Graph_Add, 2, UI_Color_Orange,4,960,120,40);//画车体UI
//			UI_Draw_Line  (&UI_Graph5.Graphic[0], "204", UI_Graph_Add, 2, UI_Color_Orange, 15, 1200, 500, 1200, 600);//画云台竖线
			UI_Draw_Line  (&UI_Graph5.Graphic[0], "205", UI_Graph_Add, 2, UI_Color_Purple, 15, 960, 120, 960, 180);//画云台竖线
			
//			UI_PushUp_Graphs(2, &UI_Graph5, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		if(UI_PushUp_Counter % 341 == 0) //动态UI预绘制 字符串1
		{
			UI_Draw_String(&UI_String.String,"203", UI_Graph_Add, 2, UI_Color_Main, 22, 8,4, 100,700, "MODE"); //摩擦轮是否开启
			UI_Draw_String(&UI_String.String,"202", UI_Graph_Add, 2, UI_Color_Main, 22, 8,4, 100,500, "M"); //摩擦轮是否开启
//			UI_PushUp_String(&UI_String, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		if(UI_PushUp_Counter % 351 == 0) //动态UI预绘制 字符串1
		{
			UI_Draw_Line (&UI_Graph2.Graphic[0], "301", UI_Graph_Add, 2, UI_Color_Purple, 15, 700, 120, 700+l5*500, 120);
//			UI_Draw_Line (&UI_Graph2.Graphic[1], "302", UI_Graph_Add, 2, UI_Color_Purple, 15, 700, 120, 700+chassis_leg[RIGHT].xB*500, 120+chassis_leg[RIGHT].yB*500);
//			UI_Draw_Line (&UI_Graph2.Graphic[2], "303", UI_Graph_Add, 2, UI_Color_Purple, 15, 700+chassis_leg[RIGHT].xB*500, 120+chassis_leg[RIGHT].yB*500,\
//																																													 700+chassis_leg[RIGHT].xC*500, 120+chassis_leg[RIGHT].yC*500);
//			UI_Draw_Line (&UI_Graph2.Graphic[3], "304", UI_Graph_Add, 2, UI_Color_Purple, 15, 700+l5*500, 120, 700+chassis_leg[RIGHT].xD*500, 120+chassis_leg[RIGHT].yD*500);
//			UI_Draw_Line (&UI_Graph2.Graphic[4], "305", UI_Graph_Add, 2, UI_Color_Purple, 15, 700+chassis_leg[RIGHT].xD*500, 120+chassis_leg[RIGHT].yD*500,\
//																																													 700+chassis_leg[RIGHT].xC*500, 120+chassis_leg[RIGHT].yC*500);
//			UI_PushUp_Graphs(5,&UI_Graph2, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		
		if(UI_PushUp_Counter % 31 == 0) //动态UI更新 字符串1
		{
			if(follow_mode){
				UI_Draw_String(&UI_String.String, "202", UI_Graph_Change, 2, UI_Color_Main,  22, 1, 3,  100, 500, "F");
			}else if(raw_mode){
				UI_Draw_String(&UI_String.String, "202", UI_Graph_Change, 2, UI_Color_Main,  22, 1, 3,  100, 500, "R");
			}else{
				UI_Draw_String(&UI_String.String, "202", UI_Graph_Change, 2, UI_Color_Main,  22, 1, 3,  100, 500, "N");
			}
//				UI_PushUp_String(&UI_String, ID_Change(Game_Robot_State.robot_id));
			continue;
		
	}

		if(UI_PushUp_Counter % 21 == 0)  //动态UI更新 图形
		{
			UI_Draw_Line (&UI_Graph2.Graphic[0], "301", UI_Graph_Change, 2, UI_Color_Purple, 15, 700, 120, 700+l5*500, 120);
//			UI_Draw_Line (&UI_Graph2.Graphic[1], "302", UI_Graph_Change, 2, UI_Color_Purple, 15, 700, 120, 700+chassis_leg[RIGHT].xB*500, 120+chassis_leg[RIGHT].yB*500);
//			UI_Draw_Line (&UI_Graph2.Graphic[2], "303", UI_Graph_Change, 2, UI_Color_Purple, 15, 700+chassis_leg[RIGHT].xB*500, 120+chassis_leg[RIGHT].yB*500,\
//																																													 700+chassis_leg[RIGHT].xC*500, 120+chassis_leg[RIGHT].yC*500);
//			UI_Draw_Line (&UI_Graph2.Graphic[3], "304", UI_Graph_Change, 2, UI_Color_Purple, 15, 700+l5*500, 120, 700+chassis_leg[RIGHT].xD*500, 120+chassis_leg[RIGHT].yD*500);
//			UI_Draw_Line (&UI_Graph2.Graphic[4], "305", UI_Graph_Change, 2, UI_Color_Purple, 15, 700+chassis_leg[RIGHT].xD*500, 120+chassis_leg[RIGHT].yD*500,\
//																																													 700+chassis_leg[RIGHT].xC*500, 120+chassis_leg[RIGHT].yC*500);
//			UI_PushUp_Graphs(5, &UI_Graph2, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		if(UI_PushUp_Counter % 6 == 0)  //动态UI更新 图形
		{
//			/* Pitch轴当前角度 */
//			UI_Draw_Float(&UI_Graph5.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, -UI_Gimbal_Pitch);
			/* 动态绘制云台UI */
			UI_Draw_Line (&UI_Graph5.Graphic[0], "205", UI_Graph_Change, 2, UI_Color_Purple, 15, 960, 120, 960+R*sin(UI_Chassis_Angle*PI/180), 120+R*cos(UI_Chassis_Angle*PI/180));
			//UI_Draw_Line  (&UI_Graph5.Graphic[1], "204", UI_Graph_Change, 2, UI_Color_Orange, 15, 960, 120, 960+R*sin(UI_Chassis_Angle*PI/180), 120+R*cos(UI_Chassis_Angle*PI/180));
//			UI_PushUp_Graphs(1, &UI_Graph5, ID_Change(Game_Robot_State.robot_id));
			continue;
		}
		vTaskDelay(10);
	}
	}
}

uint16_t this_time_rx_len = 0;
void USART6_IRQHandler(void)//接收完一帧数据后触发
{
	  HAL_UART_IRQHandler(&huart6);
	
		if(USART6->SR & UART_FLAG_IDLE)
    {
//        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
    }
}
