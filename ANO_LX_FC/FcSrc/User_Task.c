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
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
		static s32 real_X, real_Y;        // 运动距离 
		static float LPFout_x, LPFout_y;  // 位置微分滤波
		static float ms=20;
		static float fc_yaw;
		
		fc_yaw = fc_att.st_data.yaw_x100*0.01f;
    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
        //判断第6通道拨杆位置 1300<CH_6<1700，拨到中间
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //还没有执行
            if (one_key_takeoff_f == 0)
            {
                //标记已经执行
                one_key_takeoff_f =
                    //执行一键起飞
                    OneKey_Takeoff(80); //参数单位：厘米； 0：默认上位机设置的高度。
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
        }
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                one_key_land_f =
                    //执行一键降落
                    OneKey_Land();
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
        //判断第6通道拨杆位置 1700<CH_6<2000
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //还没有执行
            if (one_key_mission_f == 0)
            {
                //标记已经执行
                one_key_mission_f = 1;
                //开始流程
                mission_step = 1;
            }
        }
        else
        {
            //复位标记，以便再次执行
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
							time_dly_cnt_ms = 0;//计时器
						}
						break;
						case 1:
						{
							LX_Change_Mode(2);//切换到模式2
							
							rt_tar.st_data.vel_x = 0;//速度初始化
							rt_tar.st_data.vel_y = 0;
							// wait 1s
							if(time_dly_cnt_ms<1000)
							{
								time_dly_cnt_ms+=ms;//ms		
							}
							else
							{
								time_dly_cnt_ms = 0;
								mission_step += FC_Unlock();//解锁
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
								mission_step += OneKey_Takeoff(80);//起飞
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
