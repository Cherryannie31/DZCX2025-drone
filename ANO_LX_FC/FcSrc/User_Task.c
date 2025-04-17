#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Math.h"

#define Vel_LIMIT 12.0f
#define alpha 0

static s32 start_point_xyz[2];
// start_point(100,100)
static s32 expect_point_xy[5][2] = {{0, 150}, {100,150}, {100,250}, {50,250}, {50,200}};  // Broken line



void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //һ�����/��������
    //////////////////////////////////////////////////////////////////////
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
		static s32 real_X, real_Y;        // �˶����� 
		static float LPFout_x, LPFout_y;  // λ��΢���˲�
		static float ms=20;
		static float fc_yaw;
		
		fc_yaw = fc_att.st_data.yaw_x100*0.01f;
    //�ж���ң���źŲ�ִ��
    if (rc_in.fail_safe == 0)
    {
        //�жϵ�6ͨ������λ�� 1300<CH_6<1700�������м�
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //��û��ִ��
            if (one_key_takeoff_f == 0)
            {
                //����Ѿ�ִ��
                one_key_takeoff_f =
                    //ִ��һ�����
                    OneKey_Takeoff(80); //������λ�����ף� 0��Ĭ����λ�����õĸ߶ȡ�
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_takeoff_f = 0;
        }
        //
        //�жϵ�6ͨ������λ�� 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =
                    //ִ��һ������
                    OneKey_Land();
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
        //�жϵ�6ͨ������λ�� 1700<CH_6<2000
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //��û��ִ��
            if (one_key_mission_f == 0)
            {
                //����Ѿ�ִ��
                one_key_mission_f = 1;
                //��ʼ����
                mission_step = 1;
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_mission_f = 0;
        }
        
        if (one_key_mission_f == 1)
        {
				// Timer
					static u16 time_dly_cnt_ms;
					rt_tar.st_data.vel_x = rt_tar.st_data.vel_y = rt_tar.st_data.vel_z = rt_tar.st_data.yaw_dps = 0;
					
					switch(mission_step)
					{
						case 0: 
						{
							//reset
							time_dly_cnt_ms = 0;//��ʱ��
						}
						break;
						case 1:
						{
							LX_Change_Mode(2);//�л���ģʽ2
							
							rt_tar.st_data.vel_x = 0;//�ٶȳ�ʼ��
							rt_tar.st_data.vel_y = 0;
							// wait 1s
							if(time_dly_cnt_ms<1000)
							{
								time_dly_cnt_ms+=ms;//ms		
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += FC_Unlock();//����
								// 	Get initial position | New coordinate(0, 0)
								start_point_xyz[0] = UWB_DATA.pos_x;
								start_point_xyz[1] = UWB_DATA.pos_y;
							}		
						}
						break;
						case 2:
						{
						// wait 2s
							if(time_dly_cnt_ms<2000)
							{
								time_dly_cnt_ms+=ms;//ms
							}
							else
							{
							// Hight: 120cm
								mission_step += OneKey_Takeoff(80);//���
							}								
						}
						break;
						case 3:
						{
						// vel_z = 0
							rt_tar.st_data.vel_z = 0;
						// wait 1s
						  if(time_dly_cnt_ms<2000)
						  {
						  	time_dly_cnt_ms+=ms;//ms=20;
						  }
						  else
						  {
						  	time_dly_cnt_ms = 0;
						  	mission_step += 1;
						  }							
						}
						break;		
						case 4:
						{
							// vel_z = 0
							rt_tar.st_data.vel_z = 0;
							// Position difference differential speed output
							real_X = UWB_DATA.pos_x - start_point_xyz[0];
							real_Y = UWB_DATA.pos_y - start_point_xyz[1];
							
							S_LPF_1(0.5, (expect_point_xy[0][0] - real_X)*0.5, LPFout_x);
							S_LPF_1(0.5, (expect_point_xy[0][1] - real_Y)*0.5, LPFout_y);		
							
							rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
							rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);

							if(ABS(real_X-expect_point_xy[0][0])<10&&ABS(real_Y-expect_point_xy[0][1])<10)
							{
							// wait 1s
								if(time_dly_cnt_ms<2000)
								{
									time_dly_cnt_ms+=ms;//ms=20;
								}
								else
								{
									time_dly_cnt_ms = 0;
									mission_step += 1;
								}
							}
						}
						break;
						case 5:
						{
							// vel_z = 0
							rt_tar.st_data.vel_z = 0;
							// Position difference differential speed output
							real_X = UWB_DATA.pos_x - start_point_xyz[0];
							real_Y = UWB_DATA.pos_y - start_point_xyz[1];
							
							S_LPF_1(0.5, (expect_point_xy[1][0] - real_X)*0.5, LPFout_x);
							S_LPF_1(0.5, (expect_point_xy[1][1] - real_Y)*0.5, LPFout_y);		
							
							rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
							rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);

							if(ABS(real_X-expect_point_xy[1][0])<10&&ABS(real_Y-expect_point_xy[1][1])<10)
							{
							// wait 1s
								if(time_dly_cnt_ms<2000)
								{
									time_dly_cnt_ms+=ms;//ms=20;
								}
								else
								{
									time_dly_cnt_ms = 0;
									mission_step += 1;
								}
							}
						}
						break;
						case 6:
						{
							// vel_z = 0
							rt_tar.st_data.vel_z = 0;
				// Position difference differential speed output
							real_X = UWB_DATA.pos_x - start_point_xyz[0];
							real_Y = UWB_DATA.pos_y - start_point_xyz[1];
							
							S_LPF_1(0.5, (expect_point_xy[2][0] - real_X)*0.5, LPFout_x);
							S_LPF_1(0.5, (expect_point_xy[2][1] - real_Y)*0.5, LPFout_y);		
							
							rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
							rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);

							if(ABS(real_X-expect_point_xy[2][0])<10&&ABS(real_Y-expect_point_xy[2][1])<10)
							{
							// wait 1s
								if(time_dly_cnt_ms<2000)
								{
									time_dly_cnt_ms+=ms;//ms=20;
								}
								else
								{
									time_dly_cnt_ms = 0;
									mission_step += 1;
								}
							}
						}
						break;
						case 7:
						{
							// vel_z = 0
							rt_tar.st_data.vel_z = 0;
							// Position difference differential speed output
							real_X = UWB_DATA.pos_x - start_point_xyz[0];
							real_Y = UWB_DATA.pos_y - start_point_xyz[1];
							
							S_LPF_1(0.5, (expect_point_xy[3][0] - real_X)*0.5, LPFout_x);
							S_LPF_1(0.5, (expect_point_xy[3][1] - real_Y)*0.5, LPFout_y);		
							
							rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
							rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);

							if(ABS(real_X-expect_point_xy[3][0])<10&&ABS(real_Y-expect_point_xy[3][1])<10)
							{
							// wait 1s
								if(time_dly_cnt_ms<2000)
								{
									time_dly_cnt_ms+=ms;//ms=20;
								}
								else
								{
									time_dly_cnt_ms = 0;
									mission_step += 1;
								}
							}
						}
						break;
						case 8:
						{
							// vel_z = 0
							rt_tar.st_data.vel_z = 0;
							// Position difference differential speed output
							real_X = UWB_DATA.pos_x - start_point_xyz[0];
							real_Y = UWB_DATA.pos_y - start_point_xyz[1];
							
							S_LPF_1(0.5, (expect_point_xy[4][0] - real_X)*0.5, LPFout_x);
							S_LPF_1(0.5, (expect_point_xy[4][1] - real_Y)*0.5, LPFout_y);		
							
							rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
							rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);

							if(ABS(real_X-expect_point_xy[4][0])<10&&ABS(real_Y-expect_point_xy[4][1])<10)
							{
							// wait 1s
								if(time_dly_cnt_ms<2000)
								{
									time_dly_cnt_ms+=ms;//ms=20;
								}
								else
								{
									time_dly_cnt_ms = 0;
									mission_step += 1;
								}
							}
						}
						break;
						case 9:
						{
						// wait 2s
							if(time_dly_cnt_ms < 2000)
							{
								time_dly_cnt_ms+=ms;//ms=20;
							}
							else
							{
								OneKey_Land ();
								time_dly_cnt_ms=0;//ms=20;
								
							}
						}
						break;
						default: break;
					}
				}
		}
		else
		{
				mission_step = 0;
		}
}



























//					case 4:
//						{
//							// wait 5s to fly straight
//							if(time_dly_cnt_ms<5000)
//							{
//								time_dly_cnt_ms+=ms;//ms
//							}
//							else
//							{
//								time_dly_cnt_ms = 0;
//								new_x = UWB_DATA.pos_x-start_point_xyz[0];
//								new_y = UWB_DATA.pos_y-start_point_xyz[1];
//								rt_tar.st_data.vel_x = LIMIT((expect_point_xy[1][0]-new_x)*0.5, -Vel_LIMIT, Vel_LIMIT);
//								rt_tar.st_data.vel_y = LIMIT((expect_point_xy[1][1]-new_y)*0.5, -Vel_LIMIT, Vel_LIMIT);

//								if(ABS((UWB_DATA.pos_x-start_point_xyz[0])-expect_point_xy[0][0]) < 10
//										&&ABS((UWB_DATA.pos_y-start_point_xyz[1])-expect_point_xy[0][1]) < 10)
//								{
//									if(time_dly_cnt_ms<4000)
//									{
//									// delay 4s
//										time_dly_cnt_ms+=ms;//ms=20;
//									}
//									else
//									{
//										time_dly_cnt_ms = 0;
//									}
//								}
//									mission_step += 1;
//								}								
//						}
