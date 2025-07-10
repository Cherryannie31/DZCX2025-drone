#ifndef __USER_SEND_H
#define __USER_SEND_H

#include "SysConfig.h"
#include "Ano_Math.h"
#include "ANO_LX.h"
#include "LX_FC_State.h"
#include "mid360.h"
#include "Drv_AnoOf.h"
#include "user_control.h"

typedef struct
{
	int16_t fly_state;			//当前飞行状态 0上锁 1进入起飞倒计时 2解锁起飞 3开始程控任务 4开始降落
	int16_t LxFcState;			//凌霄飞控状态 1姿态控制模式 2定点模式 3程控模式
	int16_t bat_v100;				//电池电压 单位10MV
	int16_t CtrlMode;				//控制模式 0定点 1位置环 2绕杆 3自动降落 4程序单独控制速度
	int16_t sped_state;			//速度传感器来源 0 默认光流 1 T265 2 保留
	int16_t time_s;					//代码运行时间 单位 秒
	int16_t pos_x;					//位置 单位CM
	int16_t pos_y;					//位置 单位CM
	int16_t pos_z;					//位置 单位CM
	int16_t yaw;						//偏航角 单位度
	
}FC_data;

void FC_DateUpdate(void);
void Send_To_Board(void);
void Send_To_Analyze(void);

extern FC_data LX_FC;
// extern u8 _to_board[30];//发送数据缓冲

#endif
