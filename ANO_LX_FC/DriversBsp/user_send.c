#include "user_send.h"

FC_data LX_FC;
static u8 _to_board[30] = {0}; 


void FC_DateUpdate(void)
{
    LX_FC.LxFcState = fc_sta.fc_mode_sta;								
    LX_FC.bat_v100 = fc_bat.st_data.voltage_100;				
    // LX_FC.time_s = user_program_time/1000;							

    if(mid360_DATA.offline==0)                    
        LX_FC.sped_state=1;
    else
        LX_FC.sped_state=0;

//		LX_FC.sped_state = mid360_DATA.status;
    LX_FC.pos_x = mid360_DATA.pos_x;
    LX_FC.pos_y = mid360_DATA.pos_y;
    LX_FC.pos_z = ano_of.of_alt_cm;
    LX_FC.yaw = mid360_DATA.yaw;
}

void Send_To_Board(void)
{
		u8 send_i = 0;							// send count
		_to_board[send_i++] = 0x0A; // head 1
		_to_board[send_i++] = 0x33; // head 2
		_to_board[send_i++] = 0x34; // head 3
		_to_board[send_i++] = BYTE1(mid360_DATA.pos_x);
    _to_board[send_i++] = BYTE0(mid360_DATA.pos_x);
    _to_board[send_i++] = (uint8_t)(mid360_DATA.status);
		_to_board[send_i++] = 0x00; // tail 1
		_to_board[send_i++] = 0xF1; // tail 2
		// Trigger a send
		DrvUart3SendBuf(_to_board, send_i);
}

/**************************************************************************
函数功能：发送期望速度数据，调试pid
入口参数：
功能说明: 50ms发送，比对360数据波形
返回  值：无
作    者：
**************************************************************************/
void Send_To_Analyze(void)
{
	   	static u8 User_send_buffer[20];
      u8 cnt=0;
			
      User_send_buffer[cnt++]=0xAA;
      User_send_buffer[cnt++]=0xFF;
      User_send_buffer[cnt++]=0xF4;
      User_send_buffer[cnt++]=0x00;
      
			//期望角速度
			User_send_buffer[cnt++]=BYTE0(User_AngCtrlOutDeg);
      User_send_buffer[cnt++]=BYTE1(User_AngCtrlOutDeg);
			User_send_buffer[cnt++]=BYTE0(mid360_DATA.yaw);
      User_send_buffer[cnt++]=BYTE1(mid360_DATA.yaw);

//      height_rf= (s16)ano_of.of_alt_cm;
//      height_exp=(s16)(Exp_Alt_Zcm);
//      err=(s16)(height-height_exp);

//      loc_x_exp=Loc_Exp_Xcm;
//      lox__x_rf=(s16)(RosData.Position.x);

//      loc_y_exp=Loc_Exp_Ycm;
//      lox__y_rf=(s16)(RosData.Position.y);

//      User_send_buffer[cnt++]=BYTE0(height_exp);
//      User_send_buffer[cnt++]=BYTE1(height_exp);
//      User_send_buffer[cnt++]=BYTE0(height_rf);
//      User_send_buffer[cnt++]=BYTE1(height_rf);

//      User_send_buffer[cnt++]=BYTE0(loc_x_exp);
//      User_send_buffer[cnt++]=BYTE1(loc_x_exp);
//      User_send_buffer[cnt++]=BYTE0(lox__x_rf);
//      User_send_buffer[cnt++]=BYTE1(lox__x_rf);

//      User_send_buffer[cnt++]=BYTE0(loc_y_exp);
//      User_send_buffer[cnt++]=BYTE1(loc_y_exp);
//      User_send_buffer[cnt++]=BYTE0(lox__y_rf);
//      User_send_buffer[cnt++]=BYTE1(lox__y_rf);

//      User_send_buffer[cnt++]=BYTE0(User_AngCtrlOutDeg);
//      User_send_buffer[cnt++]=BYTE1(User_AngCtrlOutDeg);
//      User_send_buffer[cnt++]=BYTE0(LX_FC.fly_state);
//      User_send_buffer[cnt++]=BYTE1(LX_FC.fly_state);
//      User_send_buffer[cnt++]=BYTE0(RosData.CVData.x);
//      User_send_buffer[cnt++]=BYTE1(RosData.CVData.x);



      User_send_buffer[3] = cnt-4;
      //校验
			u8 check_sum1 = 0, check_sum2 = 0;
			for(u8 i=0;i<cnt;i++)
			{
				check_sum1 += User_send_buffer[i];
				check_sum2 += check_sum1;
			}
			User_send_buffer[cnt++] = check_sum1;
			User_send_buffer[cnt++] = check_sum2;
     
		  DrvUart5SendBuf(User_send_buffer,cnt);
}



