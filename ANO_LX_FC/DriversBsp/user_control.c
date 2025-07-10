#include "user_control.h"

s16 OutValue[4] = {0};	 				// ʵʱ����֡���

u8 AltCtrlEnable	=	0;				  // �߶Ȼ�ʹ��
u8 LocCtrlEnable	=	0;				  // λ�û���ʹ��
u8 AngCtrlEnable	=	0;				  // �ǶȻ�ʹ��
				
/********************	PID�����������	********************/
s16	Exp_Loc_Xcm	=	0;						//	Xλ������ֵ
s16	Exp_Loc_Ycm	=	0;						//	Yλ������ֵ		
s16	Exp_Alt_Zcm	=	0;						//	Zλ������ֵ						
s16	Exp_Ang_Deg	=	0;						//	yaw������ֵ

s16 User_LocCtrlOutXcm = 0;			//	Xλ�û������������ֵ
s16 User_LocCtrlOutYcm = 0;			//	Yλ�û������������ֵ
s16 User_AltCtrlOutZcm = 0;			//	Z�߶Ȼ������������ֵ
s16 User_AngCtrlOutDeg = 0;			//	�ǶȻ������������ֵ

s16	OutXcm = 0;								  //	���տ���֡���
s16	OutYcm = 0;								  //			
s16	OutZcm = 0;								  //		
s16	OutDeg = 0;								  //	


/**************************************************************************
�������ܣ�����״̬��ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�
**************************************************************************/
void AllInit(void)
{
	LX_FC.CtrlMode = 0;						// ����ģʽ
	
	AltCtrlEnable	=	0;						// �����ƹر�
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
�������ܣ��ɿ�ʵʱ֡����
��ڲ�����x ��x���ٶ� y��y���ٶ� z ��z���ٶ� yaw_s ����ת�ٶ�
							ע�ⵥλ		cm/s	
����  ֵ����
��    �ߣ�
**************************************************************************/
void	FC_Ctrl(s16	x, s16 y, s16	z, s16	yaw_s)
{
	OutValue[0] = x;
	OutValue[1] = y;
	OutValue[2] = z;
	OutValue[3] = yaw_s;
}


/**************************************************************************
�������ܣ�λ�û�����
��ڲ�����true_x_cm������xλ�� true_y_cm ����yλ�� exp_x_cm������x��λ�� exp_y_cm������y��λ��
����˵��: ����������ٶ����ݣ�X:User_LocCtrlOutXcm Y:User_LocCtrlOutXcm
����  ֵ����
��    �ߣ�TQ
**************************************************************************/
void Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm)
{ 
  static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 sped_out_x_cm=0,sped_out_y_cm=0;
	
	///////////////////////////////////////////////////////////////////////////////
	/*PID����*/
	err_new_x_cm=exp_x_cm-true_x_cm;
	err_new_y_cm=exp_y_cm-true_y_cm;

	err_add_x_cm+=err_new_x_cm;
	err_add_y_cm+=err_new_y_cm;

	sped_out_x_cm=(s16)(loc_ctrl_dis_p*err_new_x_cm+LIMIT(loc_ctrl_dis_i*err_add_x_cm,-10,10)+loc_ctrl_dis_d*(err_new_x_cm-err_old_x_cm));
	sped_out_y_cm=(s16)(loc_ctrl_dis_p*err_new_y_cm+LIMIT(loc_ctrl_dis_i*err_add_y_cm,-10,10)+loc_ctrl_dis_d*(err_new_y_cm-err_old_y_cm));

	err_old_x_cm=err_new_x_cm;
	err_old_y_cm=err_new_y_cm;

	//////////////////////////////////////////
	/*����*/
	sped_out_x_cm=LIMIT(sped_out_x_cm,-20,20);
	sped_out_y_cm=LIMIT(sped_out_y_cm,-20,20);
	///////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////
	//	ȡ��λ�û�
	if(LocCtrlEnable==0)
	{
	//	ע�����
		sped_out_x_cm=0;
		sped_out_y_cm=0;
	}

	User_LocCtrlOutXcm=sped_out_x_cm;
	User_LocCtrlOutYcm=sped_out_y_cm;	   
}

/**************************************************************************
�������ܣ��߶Ȼ�+�Զ�����
��ڲ�����feedback_z_com�������߶� exp_z_cm�������߶�
����˵��: �����������ȫ�ֱ������ܣ�User_AltCtrlOutZcm
����  ֵ����
��    �ߣ�TQ
**************************************************************************/
//��־λ
u8 ProgramAutoLand =0; //�̿��Զ�����
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
         /*****	����	*****/
        sped_out_z_cm=LIMIT(sped_out_z_cm,-20,20); 
     }
		 
		 //ִ���Զ�����
		 if(ProgramAutoLand==1)
      {
         LandTime+=5; 
         
				 //��ʱ���߸߶ȴﵽ������
         if(LandTime>=10*1000 || feedback_z_com<=5)
         {
            sped_out_z_cm=0;
            ProgramAutoLand=0;//�Զ������־λ��λ
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
	  
	  User_AltCtrlOutZcm = sped_out_z_cm;//�߶Ȼ��������
}

/**************************************************************************
�������ܣ��ǶȻ�����
��ڲ�����true_yaw_deg������yaw�Ƕ� exp_yaw_deg������yaw�Ƕ�
����˵��: ����ĽǶ����ݣ�User_AngCtrlOutDeg
����  ֵ����
��    �ߣ�TQ
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
      /*PID����*/
      err_new_yaw_deg=exp_yaw_deg-true_yaw_deg;	
			
			// ��С·������
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
      /*����*/
      sped_out_yaw_deg=LIMIT(sped_out_yaw_deg,-35,35);
      
			///////////////////////////////////////////////////////////////////////////////
      if(AngCtrlEnable==0)
         sped_out_yaw_deg=0;
      
			//////////////////////////////////////////////////////////////////
      User_AngCtrlOutDeg=-sped_out_yaw_deg;//���������
   }
}
