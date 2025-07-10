#include "user_send.h"

FC_data LX_FC;
static u8 _to_board[30] = {0}; 

//??????
void FC_DateUpdate(void)
{
    LX_FC.LxFcState = fc_sta.fc_mode_sta;								//????????
    LX_FC.bat_v100 = fc_bat.st_data.voltage_100;				//??????
    // LX_FC.time_s = user_program_time/1000;							//??????
    if(mid360_DATA.status == MID360_STATUS_OK)          //360????
    {
        LX_FC.sped_state=1;
    }
    else
    {
        LX_FC.sped_state=0;
    }
		LX_FC.sped_state = mid360_DATA.status;
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



