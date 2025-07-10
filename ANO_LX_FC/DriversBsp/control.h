#ifndef _CONTROL_H
#define _CONTROL_H

#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_Math.h"

//
//note:下面的环参数我都没有细调pid，后面用这个环的时候自己调一下pid参数
//

//高度环参数
#define loc_ctrl_att_p 1.00f
#define loc_ctrl_att_i 0.0f
#define loc_ctrl_att_d 0.05f

//位置环参数
#define loc_ctrl_dis_p 1.2f
#define loc_ctrl_dis_i 0.0f
#define loc_ctrl_dis_d 0.4f  

//角度环
#define loc_ctrl_ang_p 1.2f
#define loc_ctrl_ang_i 0.0f
#define loc_ctrl_ang_d 0.15f

//视觉环
#define VisualAlignment_P 0.05f
#define VisualAlignment_I 0.0f
#define VisualAlignment_D 0.01f

extern u8 ProgramAutoLand ; //程控自动降落

extern s16 CMDvalue[4]; //实时控制帧x y z deg发送数据

extern s16 User_AltCtrlOutZcm;//高度环控制输出
extern s16 User_LocCtrlOutXcm;//位置环控制输出
extern s16 User_LocCtrlOutYcm;//位置环控制输出
extern s16 User_AngCtrlOutDeg;//角度环控制输出
extern s16 VisualAlignmentOutXcm;  //X方向视觉环控制输出
extern s16 VisualAlignmentOutYcm;  //Y方向视觉环控制输出

//期望值（高度、位置、角度）在这里赋值
extern s16 Loc_Exp_Xcm;
extern s16 Loc_Exp_Ycm;
extern s16 Alt_Exp_Zcm;
extern s16 Ang_Exp_Deg;
extern s16 VisualAlignmentExpX; //视觉对齐期望值
extern s16 VisualAlignmentExpY; //视觉对齐期望值

/*最终控制量输出*/
extern s16 OutXcm;//X输出
extern s16 OutYcm;//Y输出
extern s16 OutDeg;//YAW输出
extern s16 OutZcm;//Z输出

//X Y Z使能输出 (0不控制 1控制)
extern u8 LocCtrlEnable;
extern u8 AngCtrlEnable;
extern u8 AttCtrlEnable;

void ProgramCtrl(s16 x,s16 y,s16 z,s16 dps);   //实时帧控制幅值处  单位cm/s

void RC_LOCK_Check(void);         //ANO_LX.c调用
void RC_LAND_Check(void);
void User_RC_Check(void);

void height_control(s16 feedback_z_com,s16 exp_z_cm);                                     //高度闭环
void User_Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm);                //位置闭环
void User_Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg);                                     //角度环
void User_Visual_Ctrl(s16 true_x_cm,s16 true_y_cm ,s16 exp_x_cm,s16 exp_y_cm,s16 mode);   //视觉定点环

#endif
