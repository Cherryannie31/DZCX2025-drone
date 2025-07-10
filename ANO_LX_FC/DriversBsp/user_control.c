#include "user_control.h"

s16 OutValue[4] = {0};	 				// 实时控制帧输出

u8 AltCtrlEnable	=	0;				  // 高度环使能
u8 LocCtrlEnable	=	0;				  // 位置环环使能
u8 AngCtrlEnable	=	0;				  // 角度环使能
				
/********************	PID输入输出参数	********************/
s16	Exp_Loc_Xcm	=	0;						//	X位置期望值
s16	Exp_Loc_Ycm	=	0;						//	Y位置期望值		
s16	Exp_Alt_Zcm	=	0;						//	Z位置期望值						
s16	Exp_Ang_Deg	=	0;						//	yaw轴期望值

s16 User_LocCtrlOutXcm = 0;			//	X位置环控制输出计算值
s16 User_LocCtrlOutYcm = 0;			//	Y位置环控制输出计算值
s16 User_AltCtrlOutZcm = 0;			//	Z高度环控制输出计算值
s16 User_AngCtrlOutDeg = 0;			//	角度环控制输出计算值

s16	OutXcm = 0;								  //	最终控制帧输出
s16	OutYcm = 0;								  //			
s16	OutZcm = 0;								  //		
s16	OutDeg = 0;								  //	


/**************************************************************************
函数功能：所有状态初始化
入口参数：无
返回  值：无
作    者：
**************************************************************************/
void AllInit(void)
{
	LX_FC.CtrlMode = 0;						// 定点模式
	
	AltCtrlEnable	=	0;						// 换控制关闭
	LocCtrlEnable	=	0;						//
	AngCtrlEnable	=	0;						//
	
	Exp_Loc_Xcm	=	0;
	Exp_Loc_Ycm	=	0;
	Exp_Alt_Zcm	=	0;
	Exp_Ang_Deg	=	0;
	
	User_LocCtrlOutXcm = 0;
	User_LocCtrlOutYcm = 0;
	User_AltCtrlOutZcm = 0;
	User_AngCtrlOutDeg = 0;
	
	ch_7_aux3_FLAG = 1;
}


/**************************************************************************
函数功能：飞控实时帧控制
入口参数：x ：x轴速度 y：y轴速度 z ：z轴速度 yaw_s ：旋转速度
							注意单位		cm/s	
返回  值：无
作    者：
**************************************************************************/
void	FC_Ctrl(s16	x, s16 y, s16	z, s16	yaw_s)
{
	OutValue[0] = x;
	OutValue[1] = y;
	OutValue[2] = z;
	OutValue[3] = yaw_s;
}


/**************************************************************************
函数功能：位置环控制
入口参数：true_x_cm：反馈x位置 true_y_cm 反馈y位置 exp_x_cm：期望x轴位置 exp_y_cm：期望y轴位置
功能说明: 输出的数据速度数据：X:User_LocCtrlOutXcm Y:User_LocCtrlOutXcm
返回  值：无
作    者：TQ
**************************************************************************/
void Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm)
{ 
  static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 sped_out_x_cm=0,sped_out_y_cm=0;
	
	///////////////////////////////////////////////////////////////////////////////
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
	sped_out_x_cm=LIMIT(sped_out_x_cm,-20,20);
	sped_out_y_cm=LIMIT(sped_out_y_cm,-20,20);
	///////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////
	//	取消位置环
	if(LocCtrlEnable==0)
	{
	//	注意归零
		sped_out_x_cm=0;
		sped_out_y_cm=0;
	}

	User_LocCtrlOutXcm=sped_out_x_cm;
	User_LocCtrlOutYcm=sped_out_y_cm;	   
}

/**************************************************************************
函数功能：高度环+自动降落
入口参数：feedback_z_com：反馈高度 exp_z_cm：期望高度
功能说明: 输出的数据让全局变量承受：User_AltCtrlOutZcm
返回  值：无
作    者：TQ
**************************************************************************/
//标志位
u8 ProgramAutoLand =0; //程控自动降落
void height_control(s16 feedback_z_com,s16 exp_z_cm)
{
     static float err_old_z_cm=0;
     static float err_new_z_cm=0;
     static float err_add_z_cm=0;
     static s16 sped_out_z_cm=0;

     static s16 LandTime =0;
     if(feedback_z_com<=300)
     {
        err_new_z_cm=exp_z_cm-feedback_z_com;
        err_add_z_cm+=err_new_z_cm;
        sped_out_z_cm=(s16)(loc_ctrl_att_p*err_new_z_cm\
         +LIMIT(loc_ctrl_att_i*err_add_z_cm,-10,10)\
        +loc_ctrl_att_d*(err_new_z_cm-err_old_z_cm));

        err_old_z_cm=err_new_z_cm;
         /*****	限速	*****/
        sped_out_z_cm=LIMIT(sped_out_z_cm,-20,20); 
     }
		 
		 //执行自动降落
		 if(ProgramAutoLand==1)
      {
         LandTime+=5; 
         
				 //超时或者高度达到就上锁
         if(LandTime>=10*1000 || feedback_z_com<=5)
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
    if(AltCtrlEnable==0)
    {
       sped_out_z_cm=0;
		}
	  
	  User_AltCtrlOutZcm = sped_out_z_cm;//高度环控制输出
}

/**************************************************************************
函数功能：角度环控制
入口参数：true_yaw_deg：反馈yaw角度 exp_yaw_deg：期望yaw角度
功能说明: 输出的角度数据：User_AngCtrlOutDeg
返回  值：无
作    者：TQ
**************************************************************************/
void Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg)
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
			
			// 最小路径处理
			if  (err_new_yaw_deg > 180)
					err_new_yaw_deg -= 360;
			else if (err_new_yaw_deg < 180)
					err_new_yaw_deg += 360;

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
