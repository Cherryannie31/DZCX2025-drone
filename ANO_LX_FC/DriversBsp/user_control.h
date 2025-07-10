#ifndef __USER_CONTROL_H
#define __USER_CONTROL_H

#include "SysConfig.h"
#include "Ano_Math.h"
#include "ANO_LX.h"
#include "math.h"
#include "LX_FC_Fun.h"
#include "user_send.h"
#include "User_Task.h"

//�߶Ȼ�����
#define loc_ctrl_att_p 0.30f
#define loc_ctrl_att_i 0.0f
#define loc_ctrl_att_d 0.05f

//λ�û�����
#define loc_ctrl_dis_p 1.05f
#define loc_ctrl_dis_i 0.0f
#define loc_ctrl_dis_d 0.4f  

//�ǶȻ�
#define loc_ctrl_ang_p 1.2f
#define loc_ctrl_ang_i 0.0f
#define loc_ctrl_ang_d 0.1f

extern u8 ProgramAutoLand; 			//�̿��Զ�����

extern s16 OutValue[4];	 				// ʵʱ����֡���

extern u8 AltCtrlEnable;				  // �߶Ȼ�ʹ��
extern u8 LocCtrlEnable;				  // λ�û���ʹ��
extern u8 AngCtrlEnable;				  // �ǶȻ�ʹ��
				
/********************	PID�����������	********************/
extern s16	Exp_Loc_Xcm;						//	Xλ������ֵ
extern s16	Exp_Loc_Ycm;						//	Yλ������ֵ		
extern s16	Exp_Alt_Zcm;						//	Zλ������ֵ						
extern s16	Exp_Ang_Deg;						//	yaw������ֵ

extern s16 User_LocCtrlOutXcm;			//	Xλ�û������������ֵ
extern s16 User_LocCtrlOutYcm;			//	Yλ�û������������ֵ
extern s16 User_AltCtrlOutZcm;			//	Z�߶Ȼ������������ֵ
extern s16 User_AngCtrlOutDeg;			//	�ǶȻ������������ֵ

extern s16	OutXcm;								  //	���տ���֡���
extern s16	OutYcm;								  //			
extern s16	OutZcm;								  //		
extern s16	OutDeg;								  //	

void	AllInit(void);
void FC_Ctrl(s16	x, s16 y, s16	z, s16	yaw_s);
void height_control(s16 feedback_z_com,s16 exp_z_cm);
void Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm);                    //λ�ñջ�
void Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg);

#endif
