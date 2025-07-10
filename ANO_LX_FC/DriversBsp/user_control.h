#ifndef __USER_CONTROL_H
#define __USER_CONTROL_H

#include "SysConfig.h"
#include "Ano_Math.h"
#include "ANO_LX.h"
#include "math.h"
#include "LX_FC_Fun.h"
#include "user_send.h"
#include "User_Task.h"

//高度环参数
#define loc_ctrl_att_p 0.30f
#define loc_ctrl_att_i 0.0f
#define loc_ctrl_att_d 0.05f

//位置环参数
#define loc_ctrl_dis_p 1.05f
#define loc_ctrl_dis_i 0.0f
#define loc_ctrl_dis_d 0.4f  

//角度环
#define loc_ctrl_ang_p 1.2f
#define loc_ctrl_ang_i 0.0f
#define loc_ctrl_ang_d 0.1f

extern u8 ProgramAutoLand; 			//程控自动降落

extern s16 OutValue[4];	 				// 实时控制帧输出

extern u8 AltCtrlEnable;				  // 高度环使能
extern u8 LocCtrlEnable;				  // 位置环环使能
extern u8 AngCtrlEnable;				  // 角度环使能
				
/********************	PID输入输出参数	********************/
extern s16	Exp_Loc_Xcm;						//	X位置期望值
extern s16	Exp_Loc_Ycm;						//	Y位置期望值		
extern s16	Exp_Alt_Zcm;						//	Z位置期望值						
extern s16	Exp_Ang_Deg;						//	yaw轴期望值

extern s16 User_LocCtrlOutXcm;			//	X位置环控制输出计算值
extern s16 User_LocCtrlOutYcm;			//	Y位置环控制输出计算值
extern s16 User_AltCtrlOutZcm;			//	Z高度环控制输出计算值
extern s16 User_AngCtrlOutDeg;			//	角度环控制输出计算值

extern s16	OutXcm;								  //	最终控制帧输出
extern s16	OutYcm;								  //			
extern s16	OutZcm;								  //		
extern s16	OutDeg;								  //	

void	AllInit(void);
void FC_Ctrl(s16	x, s16 y, s16	z, s16	yaw_s);
void height_control(s16 feedback_z_com,s16 exp_z_cm);
void Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm);                    //位置闭环
void Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg);

#endif
