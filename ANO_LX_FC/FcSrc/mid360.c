#include "mid360.h"
static uint8_t data_temp1[100];
u16 offline_check_time=0;
Mid360 mid360_DATA;
/**************************************************************************
函数功能：mid360数据获取
入口参数：data：串口接受数据
返回  值：无
作    者：TQ
**************************************************************************/
void mid360_GetOneByte(uint8_t data)           //数据获取
{
  static u8 _data_len = 0, _data_cnt = 0;
  static u8 rxstate = 0;

  if(rxstate==0 && data ==0xAA)               //帧头
   {
     rxstate=1;
     data_temp1[0]=data;
   }
  else if(rxstate==1 && (data == 0xff))    //目标地址
  {
    rxstate=2;
    data_temp1[1]=data;
  }
  else if(rxstate==2)                         //功能码
  {
    rxstate=3;
    data_temp1[2]=data;

  }
  else if (rxstate == 3 && data < 250)      //数据长度
  {
      rxstate = 4;
      data_temp1[3] = data;
      _data_len = data;
      _data_cnt = 0;
  }
  else if (rxstate == 4 && _data_len > 0)     //接受的数据
    {
      _data_len--;
      data_temp1[4 + _data_cnt++] = data;
      if (_data_len == 0)
          rxstate = 5;
    }
  else if(rxstate==5)                         //校验位
  {
    if(data==0x00)
    {
      Mid360_DataAnl(data_temp1,_data_cnt+4);
    }
    else
    {
      rxstate=0;
    }
  }
  else 
  {
      rxstate=0;
  }
}

/**************************************************************************
函数功能：mid360数据解析
入口参数：data_temp：串口接受数据
返回  值：无 pos_z_f:请勿乱用
作    者：TQ
**************************************************************************/
static void Mid360_DataAnl(uint8_t *data_temp,uint8_t len)
{
   if(*(data_temp+2)==0xa1)       //位置数据
   {
      int16_t x = (int16_t)((*(data_temp+4)<<8)|*(data_temp+5));
      int16_t y = (int16_t)((*(data_temp+6)<<8)|*(data_temp+7));
      int16_t z = (int16_t)((*(data_temp+8)<<8)|*(data_temp+9));

      if (!mid360_first_pose_received) {
          mid360_first_pose_received = true;
      }

      // 数据溢出判断
      if (ABS(x) < MID360_OVERFLOW_LIMIT || 
          ABS(y) < MID360_OVERFLOW_LIMIT || 
          ABS(z) < MID360_OVERFLOW_LIMIT) 
      {
          mid360_DATA.pos_x = x;
          mid360_DATA.pos_y = y;
          mid360_DATA.pos_z = z;

          mid360_DATA.pos_x_f=(int16_t)(1*(float)mid360_DATA.pos_x);
          mid360_DATA.pos_y_f=(int16_t)(1*(float)mid360_DATA.pos_y);
          mid360_DATA.pos_z_f=(int16_t)(1*(float)mid360_DATA.pos_z);

          mid360_DATA.roll=(int16_t)((*(data_temp+10)<<8)|*(data_temp+11));
          mid360_DATA.pit=(int16_t)((*(data_temp+12)<<8)|*(data_temp+13));
          mid360_DATA.yaw=(int16_t)((*(data_temp+14)<<8)|*(data_temp+15));

          mid360_DATA.v_x=(int16_t)((*(data_temp+16)<<8)|*(data_temp+17));
          mid360_DATA.v_y=(int16_t)((*(data_temp+18)<<8)|*(data_temp+19));
          mid360_DATA.v_z=(int16_t)((*(data_temp+20)<<8)|*(data_temp+21));

          // 数据正常   0
					mid360_DATA._update_cnt ++;
          mid360_status = MID360_STATUS_OK;
          mid360_overflow_cnt = 0;
      } else 
      {
        // 运行時溢出累计
        mid360_overflow_cnt++;
        if (mid360_overflow_cnt >= 2)
            // 数据溢出   1
            mid360_status = MID360_STATUS_RUNTIME_ERR;
						mid360_DATA.offline =  1;
      }
   }
   Mid360_Check_Reset();
}

/**************************************************************************
函数功能：mid360数据发送上位机
入口参数：data：串口接受数据
返回  值：无
作    者：TQ
**************************************************************************/
void Mid360send_Data()
{
      static u8 mid360_send_buffer[50];
      u8 cnt=0;
      mid360_send_buffer[cnt++]=0xAA;
      mid360_send_buffer[cnt++]=0xFF;
      mid360_send_buffer[cnt++]=0xF3;
      mid360_send_buffer[cnt++]=0x00;

      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.pos_x);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.pos_x);
      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.pos_y);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.pos_y);
      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.pos_z);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.pos_z);


      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.v_x);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.v_x);
      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.v_y);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.v_y);
      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.v_z);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.v_z);

      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.roll);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.roll);
      mid360_send_buffer[cnt++]=BYTE0(mid360_DATA.yaw);
      mid360_send_buffer[cnt++]=BYTE1(mid360_DATA.yaw);

      mid360_send_buffer[cnt++]=BYTE0(ano_of.of1_dx);
      mid360_send_buffer[cnt++]=BYTE1(ano_of.of1_dx);
      mid360_send_buffer[cnt++]=BYTE0(ano_of.of1_dy);
      mid360_send_buffer[cnt++]=BYTE1(ano_of.of1_dy);

         mid360_send_buffer[3] =  cnt - 4;
      //校验
        u8 check_sum1 = 0, check_sum2 = 0;
        for (u8 i = 0; i < cnt; i++)
        {
            check_sum1 += mid360_send_buffer[i];
            check_sum2 += check_sum1;
        }
        mid360_send_buffer[cnt++] =  check_sum1;
        mid360_send_buffer[cnt++] =  check_sum2;
     DrvUart5SendBuf(mid360_send_buffer,cnt);
}

// Function:Check the orange pie turns on or not
// 启动检测 + 溢出计时器更新
void Mid360_UpdateCheck(uint8_t dT_ms)
{
    // if (!mid360_first_pose_received) {
    //     mid360_boot_timer += dT_ms;
    //     if (mid360_boot_timer > 14000) {  
    //         // 超过14秒仍未收到有效数据
    //         mid360_status = MID360_STATUS_BOOT_FAIL;
    //     }
    // }

    // 掉綫計時
    if (offline_check_time < MID360_LOST_TIMEOUT_MS)
    {
        offline_check_time += dT_ms;
    } else{
        mid360_DATA.offline = 1;  // 掉线   2
        mid360_status = MID360_STATUS_LOST_DATA;
    }
    // 同步更新狀態
    mid360_DATA.status = mid360_status;
}

/**********************************************************************************************************
*函 数 名: Mid360_Check_Reset
*功能说明: mid360掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
void Mid360_Check_Reset(void)
{
	offline_check_time = 0;
	mid360_DATA.offline = 0;
  mid360_DATA.status = mid360_status;		// 同步更新狀態
}

