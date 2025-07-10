#ifndef _MID360_H_
#define _MID360_H_

#include "Drv_Uart.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"

#define MID360_BOOT_TIMEOUT_MS   10000  // 10 seconds
#define MID360_OVERFLOW_LIMIT    1000   // 异常值判断
#define MID360_CONT_OVERFLOW_N   5      // 连续溢出次数
#define MID360_LOST_TIMEOUT_MS   1200   // 1.2秒

// status code define
typedef enum {
    MID360_STATUS_OK = 0,          // 数据正常
    MID360_STATUS_RUNTIME_ERR,     // 数据溢出
    MID360_STATUS_LOST_DATA,       // 运行期掉线       
} Mid360_Status_e;

static uint32_t mid360_boot_timer = 0;                              // 启动超时计时器
static uint32_t mid360_lost_timer = 0;                              // 掉线计时器
static uint8_t mid360_overflow_cnt = 0;                             // 丢包计数器
static bool mid360_first_pose_received = false;                     // 首次姿态数据接收标志
static Mid360_Status_e mid360_status = MID360_STATUS_LOST_DATA;     // 状态

typedef struct
{
   int16_t pos_x;
   int16_t pos_y;
   int16_t pos_z;

   int16_t v_x;
   int16_t v_y;
   int16_t v_z;
   
   int16_t acc_x;
   int16_t acc_y;
   int16_t acc_z;
  
   int16_t pit;
   int16_t roll;
   int16_t yaw;
   
   int16_t pos_x_f;
   int16_t pos_y_f;
   int16_t pos_z_f;

   u8 offline;
   Mid360_Status_e status;
	 
	 u8 _update_cnt;       // 数据更新计数
}Mid360;

void mid360_GetOneByte(uint8_t data);           //数据获取
static void Mid360_DataAnl(uint8_t *data_temp,uint8_t len);
void Mid360send_Data(void);
void Mid360_UpdateCheck(uint8_t dT_ms);
void Mid360_Check_Reset(void);
extern Mid360 mid360_DATA;
#endif
