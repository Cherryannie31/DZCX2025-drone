#ifndef _MID360_H_
#define _MID360_H_

#include "Drv_Uart.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"

#define MID360_BOOT_TIMEOUT_MS   10000  // 10 seconds
#define MID360_OVERFLOW_LIMIT    1000   // �쳣ֵ�ж�
#define MID360_CONT_OVERFLOW_N   5      // �����������
#define MID360_LOST_TIMEOUT_MS   1200   // 1.2��

// status code define
typedef enum {
    MID360_STATUS_OK = 0,          // ��������
    MID360_STATUS_RUNTIME_ERR,     // �������
    MID360_STATUS_LOST_DATA,       // �����ڵ���       
} Mid360_Status_e;

static uint32_t mid360_boot_timer = 0;                              // ������ʱ��ʱ��
static uint32_t mid360_lost_timer = 0;                              // ���߼�ʱ��
static uint8_t mid360_overflow_cnt = 0;                             // ����������
static bool mid360_first_pose_received = false;                     // �״���̬���ݽ��ձ�־
static Mid360_Status_e mid360_status = MID360_STATUS_LOST_DATA;     // ״̬

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
	 
	 u8 _update_cnt;       // ���ݸ��¼���
}Mid360;

void mid360_GetOneByte(uint8_t data);           //���ݻ�ȡ
static void Mid360_DataAnl(uint8_t *data_temp,uint8_t len);
void Mid360send_Data(void);
void Mid360_UpdateCheck(uint8_t dT_ms);
void Mid360_Check_Reset(void);
extern Mid360 mid360_DATA;
#endif
