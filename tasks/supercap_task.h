#ifndef __SUPERCAP_TASK__H__
#define __SUPERCAP_TASK__H__
#include "stdint.h"
#include "stdbool.h"
#include "can.h"

#ifdef __GNUC__
#define DEF_PACKED_STRUCT typedef struct __attribute__((packed))
#endif

#ifdef __CC_ARM
#define DEF_PACKED_STRUCT typedef __packed struct
#endif

DEF_PACKED_STRUCT{
    uint8_t enableDCDC: 1;                  // 允许启动DCDC
    uint8_t systemRestart: 1;               // 系统重启
    uint8_t resv0: 3;
    uint8_t clearError: 1;                  // 手动清除可清除的错误
    uint8_t enableActiveChargingLimit: 1;   // 是否启用主动充电限制
    uint8_t useNewFeedbackMessage: 1;       // 是否使用新的反馈消息格式
    uint16_t refereePowerLimit;             // 裁判限制功率，单位W
    uint16_t refereeEnergyBuffer;           // 裁判能量缓冲，单位J
    uint8_t activeChargingLimitRatio;       // 主动充电限制比例（能量），0-255
    int16_t resv2;
} SetReport;

DEF_PACKED_STRUCT{               // 0x052 (useNewFeedbackMessage = 1)
    uint8_t statusCode;             // 状态信息
    uint16_t chassisPower;          // 底盘功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    uint16_t refereePower;          // 裁判系统功率，功率*64+16384 (-256W~+768W, 精度0.015625)
    uint16_t chassisPowerLimit;     // 底盘最大可用功率（包括裁判系统）
    uint8_t capEnergy;              // 电容现有能量，0-255
} StatusReport;

/* 以上两个结构体只用于内部实现，无需关注 */

/* 超电报告错误等级 ERROR_RECOVER_MANUAL 错误需要根据情况决定是否调用clean_error()清除 */
typedef enum{
    NO_ERROR = 0,               // 无错误
    ERROR_RECOVER_AUTO = 1,     // 错误，可通过自动恢复
    ERROR_RECOVER_MANUAL = 2,   // 错误，可通过发信息恢复
    ERROR_UNRECOVERABLE = 3,    // 错误，不可恢复
    WARNING                     // 警告
} ErrorLevel;

/* 超电报告当前限流原因 */
typedef enum{   
    REFEREE_POWER = 0,
    CAPARR_VOLTAGE_MAX,
    CAPARR_VOLTAGE_NORMAL,
    IB_POSITIVE,
    IB_NEGATIVE,
} LimitFactor;

/* 超电汇报数据的类型,不建议使用，推荐直接用下面的宏定义来读值 */
typedef enum{
    AB_ENABLE = 0,
    ENERGY,
    LIMIT_REASON,
    CHASSISPOWER,
    REFEREEPOWER,
    CHASSISPOWERLIMIT,
    ERR_LEVEL
} valueTypes_t;

/* 初始化超电通讯, config_can代表是否帮助配置can外设，如果can外设不是超电专属的can外设，就传false，否则传true */
void initialize_communication(bool config_can);

/* 超电通讯的tick函数，需要放在死循环中执行，该函数负责更新各种数值到超电，并更新超电汇报的状态数据 */
void com_onTick();

/* 超电can总线接收回调, 需要将其放在对应的FifoMsgPending回调函数中, rx_fifo是指定哪个fifo是接收超电消息的,如:CAN_RX_FIFO0 */
void spcom_handleCanMsg(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData);

/* 读取特定数值，不建议使用，推荐用下面的宏定义*/
uint16_t readValue(valueTypes_t vt);

/* 设置是否允许DCDC true为允许, false为不允许 */
void set_DCDC_Status(bool status);

/* 设置充电比例限制, ratio必须是0到1之间的数，超过限制会强制约束 */
void set_charge_limit(float ratio);

/* 设置裁判系统能量缓冲，以焦耳为单位 */
void set_refereeEnergyBuffer(uint16_t ene);

/* 设置裁判系统功率限制，以瓦特为单位 */
void set_refereePowerLimit(uint16_t power);

/* 请求重启超电模块 */
void restart_supercap();

/* 请求清除超电错误状态*/
void clean_error();

/* 关闭充电比例限制 */
void disable_charge_limit();

/* 对以上接口的额外补充说明:
 * 1.设置充电比例限制后，如果调用disable_charge_limit则充电比例限制关闭，若需重新开启，需要重新调用set_charge_limit
 * 2.不建议使用readValue + valueTypes_t的方式读取数值，这需要手动转化数据单位和数据类型，而宏定义则已经转化好了
*/


/* 获取超电错误等级，请查看ErrorLevel枚举 */
#define Get_ERRLEVEL() ((ErrorLevel)readValue(ERR_LEVEL))

/* 获取DCDC开启状态, true为开 false为关 */
#define IS_AB_ENABLE() ((bool)readValue(AB_ENABLE))

/* 获取超电剩余电量比例, 该数值在超电模块里已经进行了非线性映射，百分比指的是能量百分比，而非电压 */
#define Get_ENERGY() ((double)(readValue(ENERGY) / 250.0f))

/* 获取限流原因，请查看LimitFactor枚举 */
#define Get_LIMITREASON() ((LimitFactor)readValue(LIMIT_REASON))

/* 获取底盘最大可用功率，该数值由超电自动计算: 超级电容能提供的功率 + 裁判系统能提供的功率 */
#define Get_CHASSISPOWERMAX() readValue(CHASSISPOWERLIMIT)

/* 获取底盘当前消耗功率 */
#define Get_CHASSISPOWER() ((double)((readValue(CHASSISPOWER) - 16384) / 64.0f))

/* 获取裁判系统当前输入功率 */
#define Get_REFEREEPOWER() ((double)((readValue(REFEREEPOWER) - 16384) / 64.0f))

/* 注意:
 * 上述所有接口函数全部不支持在中断上下文中调用，请勿尝试
 * communication.c第10行的宏定义需要根据cube里配置的can外设修改
 * 读值依靠超电模块按一定频率自主上报，故可能存在一定延迟，避免在修改数据后，紧接着就开始读值，若数据处理对时间敏感，建议延时后读取
 * 超电模块上报频率: 1kHz 故延迟应大于等于1ms
 * 对于函数返回值和参数的单位制，未提及均为国际主单位
 * 该部分代码只负责通讯, 各种数值设定后会造成什么影响，不是该部分代码决定的，请先查看超电开源手册https://bbs.robomaster.com/article/761385?source=4
 */

/* 调用顺序问题:
 * 在最开始的时候，需要先调用initialize_communication(bool)进行初始化
 * 在can外设对应FIFO的回调函数中执行spcom_RxFifoMsgPendingCallback(CAN_HandleTypeDef *hcan, uint32_t rx_fifo)，其内部会处理can报文
 * 初始化后可以设置数值，但不会被同步到超电，若不想让第一轮数据同步默认值，可在com_onTick()第一次执行之前设置值
 * 然后在死循环里执行com_onTick(), 该函数需要一直被重复执行
 * com_onTick()重复执行后，才可以进行读取操作，否则读值全部为默认值
*/

/* 示例:
 * 本项目freertos.c:140 创建任务，初始化并循环执行com_onTick
 * main.c:60 执行canrx回调spcom_RxFifoMsgPendingCallback(CAN_HandleTypeDef *hcan, uint32_t rx_fifo)
 * 注意: 读取和写入，没有严格的顺序要求，只要com_onTick在循环执行，就没问题
*/

void supercap_task(void const *argument);

#endif

