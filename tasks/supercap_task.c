#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "supercap_task.h"
#include "referee.h"

#define powerCan hcan2
#define confine(target, max, min) ((target > max) ? max : ((target < min) ? min : target)) // 限制值




volatile uint16_t energy_raw ;
volatile double energy_get ;

void supercap_task(void const *argument){
//	
	initialize_communication(false);
  /* Infinite loop */
  for(;;)
  {
    set_refereeEnergyBuffer(Power_Heat_Data.buffer_energy);
//		set_refereeEnergyBuffer(eb);
    set_refereePowerLimit(Game_Robot_State.chassis_power_limit);
//		set_refereePowerLimit(pl);
    com_onTick(); // 请求提交并更新超电状态数据
		energy_raw = readValue(ENERGY);
    energy_get = Get_ENERGY();
    osDelay(100);
  }
	
}









static xQueueHandle rec_queue = NULL;

static bool outputABEnabled = false;

static uint8_t errorlev = NO_ERROR;
static uint8_t limitFactor = REFEREE_POWER;
static uint8_t energy = 0;

static uint16_t chassisPower = 0;
static uint16_t refereePower = 0;
static uint16_t chassisPowerLimit = 0;

static bool is_initialized = false;



static SetReport set_report = {
    .activeChargingLimitRatio = 255,
    .clearError = 0,
    .enableActiveChargingLimit = 0,
    .enableDCDC = 1,
    .useNewFeedbackMessage = 1,
    .systemRestart = 0,
    .refereeEnergyBuffer = 57,
    .refereePowerLimit = 50
};


void initialize_communication(bool config_can){
    if(config_can){
        CAN_FilterTypeDef filter = {
            .FilterActivation = ENABLE,
            .FilterBank = 0,
            .FilterFIFOAssignment = CAN_FILTER_FIFO0,
            .FilterIdHigh = (0x052 << 5),
            // .FilterIdHigh = 0x000,
            .FilterIdLow = 0x0000,
            .FilterMaskIdHigh = (0x7FF << 5),
            // .FilterMaskIdHigh = 0x000,
            .FilterMaskIdLow = 0x0000,
            .FilterMode = CAN_FILTERMODE_IDMASK,
            .FilterScale = CAN_FILTERSCALE_32BIT,
        };
        HAL_CAN_ConfigFilter(&powerCan, &filter);
        HAL_CAN_ActivateNotification(&powerCan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
        HAL_CAN_Start(&powerCan);
    }
    rec_queue = xQueueCreate(2, sizeof(StatusReport));

    is_initialized = true;
}

static void sendCanReport(uint32_t id, const char* data, uint16_t len){
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.StdId = id;           // 标准ID
    txHeader.ExtId = 0x00;         // 扩展ID
    txHeader.IDE = CAN_ID_STD;     // 标准帧
    txHeader.RTR = CAN_RTR_DATA;   // 数据帧
    txHeader.DLC = len;            // 数据长度
    txHeader.TransmitGlobalTime = DISABLE;
    volatile HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&powerCan, &txHeader, (uint8_t*)data, &txMailbox);
}


void com_onTick(void* params)
{
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用

    static uint32_t last_tick = 0;

    if(!is_initialized) return; // 未初始化禁止调用tick

    if(xTaskGetTickCount() - last_tick > 100){
				taskENTER_CRITICAL();  
				sendCanReport(0x061, (const char *)&set_report, sizeof set_report);				      
        if(set_report.systemRestart == 1) set_report.systemRestart = 0; // 发送完一次重启，清零重启标志位
        if(set_report.clearError == 1) set_report.clearError = 0; // 发送完一次清除错误，清零清除错误标志位
				taskEXIT_CRITICAL();
        last_tick = xTaskGetTickCount();
    }

    StatusReport report = {0};
    if(xQueueReceive(rec_queue, &report, 0)){
        taskENTER_CRITICAL(); // 共享变量，原子操作
        errorlev = (uint8_t)(report.statusCode & 0x03);
        outputABEnabled = (report.statusCode & 0x40) ? true : false;
        limitFactor = (uint8_t)((report.statusCode >> 2) & 0xFF);
        energy = report.capEnergy;
        chassisPower = report.chassisPower;
        refereePower = report.refereePower;
        chassisPowerLimit = report.chassisPowerLimit;
        taskEXIT_CRITICAL();
    }
}

void spcom_handleCanMsg(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData){
    if(!is_initialized) return;
    if(hcan == &powerCan){
        if(rxHeader->DLC == sizeof(StatusReport) && rxHeader->StdId == 0x052){
            xQueueSendFromISR(rec_queue, rxData, 0);
        }
    }
}

uint16_t readValue(valueTypes_t vt){

    if(xPortIsInsideInterrupt() == pdTRUE) return 0; // 禁止在中断中读取

    volatile uint16_t result = 0;

    taskENTER_CRITICAL(); // 原子读取
    switch (vt){
        case AB_ENABLE:
            result = outputABEnabled;
						break;
        case ENERGY:
            result = energy;
						break;
        case LIMIT_REASON:
            result = limitFactor;
						break;
        case CHASSISPOWER:
            result = chassisPower;
						break;
        case REFEREEPOWER:
            result = refereePower;
						break;
        case CHASSISPOWERLIMIT:
            result = chassisPowerLimit;
						break;
        case ERR_LEVEL:
            result = errorlev;
						break;
        default:
            result = 0;
						break;
    }
    taskEXIT_CRITICAL();
    return result;
}

void set_DCDC_Status(bool status){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.enableDCDC = (uint8_t)status;

    taskEXIT_CRITICAL();
}

void set_charge_limit(float ratio){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.enableActiveChargingLimit = 1;
    set_report.activeChargingLimitRatio = (uint8_t)confine(ratio * 255.0f, 255, 0);

    taskEXIT_CRITICAL();
}

void disable_charge_limit(){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.enableActiveChargingLimit = 0;
    set_report.activeChargingLimitRatio = 255;

    taskEXIT_CRITICAL();
}

void set_refereeEnergyBuffer(uint16_t ene){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.refereeEnergyBuffer = ene;

    taskEXIT_CRITICAL();
}

void set_refereePowerLimit(uint16_t power){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.refereePowerLimit = power;

    taskEXIT_CRITICAL();
}

void restart_supercap(){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.systemRestart = 1;

    taskEXIT_CRITICAL();
}

void clean_error(){
    if(xPortIsInsideInterrupt() == pdTRUE) return; // 禁止在中断中调用
    taskENTER_CRITICAL();

    set_report.clearError = 1;

    taskEXIT_CRITICAL();
}



