#ifndef _MID360_H_
#define _MID360_H_

#include "Drv_Uart.h"
#include "Drv_AnoOf.h"

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
}Mid360;

void mid360_GetOneByte(uint8_t data);           //数据获取
static void Mid360_DataAnl(uint8_t *data_temp,uint8_t len);
void Mid360send_Data(void);
extern Mid360 mid360_DATA;
#endif
