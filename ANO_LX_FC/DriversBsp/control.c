#include "control.h"

s16 CMDvalue[4]={0}; //ʵʱ����֡x y z deg��������  

//��־λ
u8 ProgramAutoLand =0; //�̿��Զ�����

s16 User_AltCtrlOutZcm=0;//�߶Ȼ��������
s16 User_LocCtrlOutXcm=0;//Xλ�û��������
s16 User_LocCtrlOutYcm=0;//Yλ�û��������
s16 User_AngCtrlOutDeg=0;//�ǶȻ��������
s16 VisualAlignmentOutXcm=0;  //X�����Ӿ����������
s16 VisualAlignmentOutYcm=0;  //Y�����Ӿ����������

//����ֵ���߶ȡ�λ�á��Ƕȣ������︳ֵ
s16 Loc_Exp_Xcm =0;
s16 Loc_Exp_Ycm =0;
s16 Alt_Exp_Zcm =0;
s16 Ang_Exp_Deg =0;
s16 VisualAlignmentExpX=0; //�Ӿ���������ֵ
s16 VisualAlignmentExpY=0; //�Ӿ���������ֵ

/*���տ��������*/
s16 OutXcm=0;//X���
s16 OutYcm=0;//Y���
s16 OutDeg=0;//YAW���
s16 OutZcm=0;//Z���

/*����ʹ�ܱ�־λ0������ 1 ����*/
u8 LocCtrlEnable =0;   //λ��ʹ�����
u8 AngCtrlEnable =0;   //�Ƕ�ʹ�����
u8 AttCtrlEnable =0;   //�߶�ʹ�����

/**************************************************************************
�������ܣ��ɿ�ʵʱ֡����
��ڲ�����x ��x���ٶ� y��y���ٶ� z ��z���ٶ� dps ����ת�ٶ�
����  ֵ����
��    �ߣ�TQ
**************************************************************************/
void ProgramCtrl(s16 x,s16 y,s16 z,s16 dps)   //ʵʱ֡���Ʒ�ֵ��  ��λcm/s
{
   CMDvalue[0] =  x;
	CMDvalue[1] =  y;
	CMDvalue[2]=   z;
	CMDvalue[3] = dps;
}

/**************************************************************************
�������ܣ��߶Ȼ�+�Զ�����
��ڲ�����feedback_z_com�������߶� exp_z_cm�������߶�
����˵��: �����������ȫ�ֱ������ܣ�User_AltCtrlOutZcm
����  ֵ����
��    �ߣ�TQ
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
         /*����*/
        sped_out_z_cm=LIMIT(sped_out_z_cm,-35,35); 
      if(AttCtrlEnable==0)
      {
         sped_out_z_cm=0;
      }
      if(ProgramAutoLand==1)//ִ���Զ�����
      {
         LandTime+=5; 
         
         if(LandTime>=10*1000 || feedback_z_com<=5)//��ʱ���߸߶ȴﵽ������
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
	  User_AltCtrlOutZcm=sped_out_z_cm;//�߶Ȼ��������
}

/**************************************************************************
�������ܣ�λ�û�����
��ڲ�����true_x_cm������xλ�� true_y_cm ����yλ�� exp_x_cm������x��λ�� exp_y_cm������y��λ��
����˵��: ����������ٶ����ݣ�X:User_LocCtrlOutXcm Y:User_LocCtrlOutXcm
����  ֵ����
��    �ߣ�TQ
**************************************************************************/
void User_Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm)                     //λ�ñջ�
{
   static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 sped_out_x_cm=0,sped_out_y_cm=0;
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
�������ܣ��ǶȻ�����
��ڲ�����true_yaw_deg������yaw�Ƕ� exp_yaw_deg������yaw�Ƕ�
����˵��: ����ĽǶ����ݣ�User_AngCtrlOutDeg
����  ֵ����
��    �ߣ�TQ
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
      /*PID����*/
      err_new_yaw_deg=exp_yaw_deg-true_yaw_deg;	

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
/**************************************************************************
�������ܣ��Ӿ����㻷����
��ڲ�����true_x_cm������xλ�� true_y_cm:����yλ�� exp_x_cm������x�Ƕ� exp_y_cm ����y�Ƕ�  mode:ʹ�����
����˵��: ������Ӿ������ݣ�VisualAlignmentOutXcm  VisualAlignmentOutYcm
����  ֵ����
��    �ߣ�TQ
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

	/*����*/
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
�������ܣ��û�ң�����������
��ڲ�����
����  ֵ��
��    �ߣ�TQ
**************************************************************************/
void User_RC_Check(void)
{
	RC_LOCK_Check();
	RC_LAND_Check();
}
/**************************************************************************
�������ܣ��ɿ�һ������
��ڲ�����
����  ֵ��
��    �ߣ�TQ
**************************************************************************/
void RC_LOCK_Check(void)//ANO_LX.c����
{
	/*�ж�ң�������Ͻǰ�ť�Ƿ񱻴���*/
	if(rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1400)
	{
		FC_Lock();/*����*/
	}
}
/**************************************************************************
�������ܣ��ɻ�һ������
��ڲ�����
����  ֵ��
��    �ߣ�TQ
**************************************************************************/
void RC_LAND_Check(void)
{
	static uint8_t i=0;/*�Ƿ�ִ�б�־λ*/
	/*�ж�ң�������Ͻǰ�ť�Ƿ񱻴���*/
	if(rc_in.rc_ch.st_data.ch_[ch_8_aux4]>1400)
	{
		if(i==0)
		{	
			i=OneKey_Land();/*һ�����䣬�ɹ�ִ��һ�κ���ִ��*/
		}
	}
	else
		i=0;/*��־λ��λ*/
}

