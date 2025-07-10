#include "control.h"

s16 CMDvalue[4]={0}; //实时控制帧x y z deg发送数据  

//标志位
u8 ProgramAutoLand =0; //程控自动降落

s16 User_AltCtrlOutZcm=0;//高度环控制输出
s16 User_LocCtrlOutXcm=0;//X位置环控制输出
s16 User_LocCtrlOutYcm=0;//Y位置环控制输出
s16 User_AngCtrlOutDeg=0;//角度环控制输出
s16 VisualAlignmentOutXcm=0;  //X方向视觉环控制输出
s16 VisualAlignmentOutYcm=0;  //Y方向视觉环控制输出

//期望值（高度、位置、角度）在这里赋值
s16 Loc_Exp_Xcm =0;
s16 Loc_Exp_Ycm =0;
s16 Alt_Exp_Zcm =0;
s16 Ang_Exp_Deg =0;
s16 VisualAlignmentExpX=0; //视觉对齐期望值
s16 VisualAlignmentExpY=0; //视觉对齐期望值

/*最终控制量输出*/
s16 OutXcm=0;//X输出
s16 OutYcm=0;//Y输出
s16 OutDeg=0;//YAW输出
s16 OutZcm=0;//Z输出

/*控制使能标志位0不控制 1 控制*/
u8 LocCtrlEnable =0;   //位置使能输出
u8 AngCtrlEnable =0;   //角度使能输出
u8 AttCtrlEnable =0;   //高度使能输出

/**************************************************************************
函数功能：飞控实时帧控制
入口参数：x ：x轴速度 y：y轴速度 z ：z轴速度 dps ：旋转速度
返回  值：无
作    者：TQ
**************************************************************************/
void ProgramCtrl(s16 x,s16 y,s16 z,s16 dps)   //实时帧控制幅值处  单位cm/s
{
   CMDvalue[0] =  x;
	CMDvalue[1] =  y;
	CMDvalue[2]=   z;
	CMDvalue[3] = dps;
}

/**************************************************************************
函数功能：高度环+自动降落
入口参数：feedback_z_com：反馈高度 exp_z_cm：期望高度
功能说明: 输出的数据让全局变量承受：User_AltCtrlOutZcm
返回  值：无
作    者：TQ
**************************************************************************/
void height_control(s16 feedback_z_com,s16 exp_z_cm)
{
     static float err_old_z_cm=0;
     static float err_new_z_cm=0;
     static float err_add_z_cm=0;
     static s16 sped_out_z_cm=0;

	  static s16 LandTime =0;

     err_new_z_cm=exp_z_cm-feedback_z_com;
     err_add_z_cm+=err_new_z_cm;
     sped_out_z_cm=(s16)(loc_ctrl_att_p*err_new_z_cm\
      +LIMIT(loc_ctrl_att_i*err_add_z_cm,-10,10)\
     +loc_ctrl_att_d*(err_new_z_cm-err_old_z_cm));

     err_old_z_cm=err_new_z_cm;
         /*限速*/
        sped_out_z_cm=LIMIT(sped_out_z_cm,-35,35); 
      if(AttCtrlEnable==0)
      {
         sped_out_z_cm=0;
      }
      if(ProgramAutoLand==1)//执行自动降落
      {
         LandTime+=5; 
         
         if(LandTime>=10*1000 || feedback_z_com<=5)//超时或者高度达到就上锁
         {
            sped_out_z_cm=0;
            ProgramAutoLand=0;//自动降落标志位复位
            LandTime=0;
            FC_Lock();
         }
         if(feedback_z_com>=100)
            sped_out_z_cm=-30;
         else if(feedback_z_com>=30)
            sped_out_z_cm=-15;
         else 
            sped_out_z_cm=-9;
      }
	  User_AltCtrlOutZcm=sped_out_z_cm;//高度环控制输出
}

/**************************************************************************
函数功能：位置环控制
入口参数：true_x_cm：反馈x位置 true_y_cm 反馈y位置 exp_x_cm：期望x轴位置 exp_y_cm：期望y轴位置
功能说明: 输出的数据速度数据：X:User_LocCtrlOutXcm Y:User_LocCtrlOutXcm
返回  值：无
作    者：TQ
**************************************************************************/
void User_Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm)                     //位置闭环
{
   static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 sped_out_x_cm=0,sped_out_y_cm=0;
   /*PID计算*/
	err_new_x_cm=exp_x_cm-true_x_cm;
	err_new_y_cm=exp_y_cm-true_y_cm;

	err_add_x_cm+=err_new_x_cm;
	err_add_y_cm+=err_new_y_cm;

	sped_out_x_cm=(s16)(loc_ctrl_dis_p*err_new_x_cm+LIMIT(loc_ctrl_dis_i*err_add_x_cm,-10,10)+loc_ctrl_dis_d*(err_new_x_cm-err_old_x_cm));
	sped_out_y_cm=(s16)(loc_ctrl_dis_p*err_new_y_cm+LIMIT(loc_ctrl_dis_i*err_add_y_cm,-10,10)+loc_ctrl_dis_d*(err_new_y_cm-err_old_y_cm));

	err_old_x_cm=err_new_x_cm;
	err_old_y_cm=err_new_y_cm;

	//////////////////////////////////////////
	/*限速*/
	sped_out_x_cm=LIMIT(sped_out_x_cm,-25,25);
	sped_out_y_cm=LIMIT(sped_out_y_cm,-25,25);
	///////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////
	if(LocCtrlEnable==0)
	{
		sped_out_x_cm=0;
		sped_out_y_cm=0;
	}

	User_LocCtrlOutXcm=sped_out_x_cm;
	User_LocCtrlOutYcm=sped_out_y_cm;
	///////////////////////////////////////////////////////////////////////////////
	   
}
/**************************************************************************
函数功能：角度环控制
入口参数：true_yaw_deg：反馈yaw角度 exp_yaw_deg：期望yaw角度
功能说明: 输出的角度数据：User_AngCtrlOutDeg
返回  值：无
作    者：TQ
**************************************************************************/
void User_Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg)
{
	static float err_old_yaw_deg=0;
	static float err_new_yaw_deg=0;
	static float err_add_yaw_deg=0;
	static s16 sped_out_yaw_deg=0;
	///////////////////////////////////////////////////////////////////////////////
   if(ABS(true_yaw_deg)>180){}
   else if(true_yaw_deg>=-180&&true_yaw_deg<=180)
   {
      /*PID计算*/
      err_new_yaw_deg=exp_yaw_deg-true_yaw_deg;	

      err_add_yaw_deg+=err_new_yaw_deg;

      sped_out_yaw_deg=(s16)(loc_ctrl_ang_p*err_new_yaw_deg\
      +LIMIT(loc_ctrl_ang_i*err_add_yaw_deg,-10,10)\
      +loc_ctrl_ang_d*(err_new_yaw_deg-err_old_yaw_deg));

      err_old_yaw_deg=err_new_yaw_deg;
      //////////////////////////////////////////
      /*限速*/
      sped_out_yaw_deg=LIMIT(sped_out_yaw_deg,-35,35);
      ///////////////////////////////////////////////////////////////////////////////
      if(AngCtrlEnable==0)
         sped_out_yaw_deg=0;
      //////////////////////////////////////////////////////////////////
      User_AngCtrlOutDeg=-sped_out_yaw_deg;//控制量输出
   }

}
/**************************************************************************
函数功能：视觉定点环控制
入口参数：true_x_cm：反馈x位置 true_y_cm:反馈y位置 exp_x_cm：期望x角度 exp_y_cm 期望y角度  mode:使能输出
功能说明: 输出的视觉环数据：VisualAlignmentOutXcm  VisualAlignmentOutYcm
返回  值：无
作    者：TQ
**************************************************************************/
void User_Visual_Ctrl(s16 true_x_cm,s16 true_y_cm ,s16 exp_x_cm,s16 exp_y_cm,s16 mode)
{
	static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 spd_out_x_cm=0,spd_out_y_cm=0;
	///////////////////////////////////////////////////////////////////////////////
   err_new_x_cm=exp_x_cm-true_x_cm;
	err_new_y_cm=exp_y_cm-true_y_cm;

   err_add_x_cm+=err_new_x_cm;
	err_add_y_cm+=err_new_y_cm;

   spd_out_x_cm=(s16)( VisualAlignment_P*err_new_x_cm+LIMIT(VisualAlignment_I*err_add_x_cm,-10,10)+VisualAlignment_D*(err_new_x_cm-err_old_x_cm));
	spd_out_y_cm=(s16)( VisualAlignment_P*err_new_y_cm+LIMIT(VisualAlignment_I*err_add_y_cm,-10,10)+VisualAlignment_D*(err_new_y_cm-err_old_y_cm));

	err_old_x_cm=err_new_x_cm;
	err_old_y_cm=err_new_y_cm;

	/*限速*/
	spd_out_x_cm=LIMIT(spd_out_x_cm,-10,10);
	spd_out_y_cm=LIMIT(spd_out_y_cm,-10,10);

	if(mode==0)
	{
	spd_out_x_cm=0;
	spd_out_y_cm=0;
	}

	VisualAlignmentOutXcm=spd_out_y_cm;
	VisualAlignmentOutYcm=spd_out_x_cm;
}

/**************************************************************************
函数功能：用户遥控器按键检测
入口参数：
返回  值：
作    者：TQ
**************************************************************************/
void User_RC_Check(void)
{
	RC_LOCK_Check();
	RC_LAND_Check();
}
/**************************************************************************
函数功能：飞控一键上锁
入口参数：
返回  值：
作    者：TQ
**************************************************************************/
void RC_LOCK_Check(void)//ANO_LX.c调用
{
	/*判断遥控器右上角按钮是否被打下*/
	if(rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1400)
	{
		FC_Lock();/*上锁*/
	}
}
/**************************************************************************
函数功能：飞机一键降落
入口参数：
返回  值：
作    者：TQ
**************************************************************************/
void RC_LAND_Check(void)
{
	static uint8_t i=0;/*是否执行标志位*/
	/*判断遥控器左上角按钮是否被打下*/
	if(rc_in.rc_ch.st_data.ch_[ch_8_aux4]>1400)
	{
		if(i==0)
		{	
			i=OneKey_Land();/*一键降落，成功执行一次后不再执行*/
		}
	}
	else
		i=0;/*标志位复位*/
}

