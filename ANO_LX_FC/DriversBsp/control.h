#ifndef _CONTROL_H
#define _CONTROL_H

#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_Math.h"

//
//note:����Ļ������Ҷ�û��ϸ��pid���������������ʱ���Լ���һ��pid����
//

//�߶Ȼ�����
#define loc_ctrl_att_p 1.00f
#define loc_ctrl_att_i 0.0f
#define loc_ctrl_att_d 0.05f

//λ�û�����
#define loc_ctrl_dis_p 1.2f
#define loc_ctrl_dis_i 0.0f
#define loc_ctrl_dis_d 0.4f  

//�ǶȻ�
#define loc_ctrl_ang_p 1.2f
#define loc_ctrl_ang_i 0.0f
#define loc_ctrl_ang_d 0.15f

//�Ӿ���
#define VisualAlignment_P 0.05f
#define VisualAlignment_I 0.0f
#define VisualAlignment_D 0.01f

extern u8 ProgramAutoLand ; //�̿��Զ�����

extern s16 CMDvalue[4]; //ʵʱ����֡x y z deg��������

extern s16 User_AltCtrlOutZcm;//�߶Ȼ��������
extern s16 User_LocCtrlOutXcm;//λ�û��������
extern s16 User_LocCtrlOutYcm;//λ�û��������
extern s16 User_AngCtrlOutDeg;//�ǶȻ��������
extern s16 VisualAlignmentOutXcm;  //X�����Ӿ����������
extern s16 VisualAlignmentOutYcm;  //Y�����Ӿ����������

//����ֵ���߶ȡ�λ�á��Ƕȣ������︳ֵ
extern s16 Loc_Exp_Xcm;
extern s16 Loc_Exp_Ycm;
extern s16 Alt_Exp_Zcm;
extern s16 Ang_Exp_Deg;
extern s16 VisualAlignmentExpX; //�Ӿ���������ֵ
extern s16 VisualAlignmentExpY; //�Ӿ���������ֵ

/*���տ��������*/
extern s16 OutXcm;//X���
extern s16 OutYcm;//Y���
extern s16 OutDeg;//YAW���
extern s16 OutZcm;//Z���

//X Y Zʹ����� (0������ 1����)
extern u8 LocCtrlEnable;
extern u8 AngCtrlEnable;
extern u8 AttCtrlEnable;

void ProgramCtrl(s16 x,s16 y,s16 z,s16 dps);   //ʵʱ֡���Ʒ�ֵ��  ��λcm/s

void RC_LOCK_Check(void);         //ANO_LX.c����
void RC_LAND_Check(void);
void User_RC_Check(void);

void height_control(s16 feedback_z_com,s16 exp_z_cm);                                     //�߶ȱջ�
void User_Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm);                //λ�ñջ�
void User_Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg);                                     //�ǶȻ�
void User_Visual_Ctrl(s16 true_x_cm,s16 true_y_cm ,s16 exp_x_cm,s16 exp_y_cm,s16 mode);   //�Ӿ����㻷

#endif
