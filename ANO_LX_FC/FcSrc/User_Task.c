#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Ano_Math.h"

#define Height_80cm 80
#define Height_100cm_test 120
#define Vel_LIMIT 12.0f
#define alpha 0

u8 ch_7_aux3_FLAG = 0;

static s32 start_point_xyz[2];
// start_point(100,100)
static s32 expect_point_xy[5][2] = {{0, 150}, {100, 150}, {100, 250}, {20, 250}, {20, 20}}; // Broken line
static s32 expect_point_xy_1[5][2] = {{90, 0}, {180, 0}, {180, 150}, {30, 150}, {30, 50}};	// Broken line

Exp_Point_Typedef Exp_Point[50] =
		{
				// x	y	 z	yaw  arrive
				{0, 0, Height_80cm, 0, 0},					 //	0
				{90, 0, Height_80cm, 0, 0},					 //	1
				{180, 0, Height_100cm_test, 0, 0},	 //	2
				{180, 150, Height_100cm_test, 0, 0}, //	3
				{30, 150, Height_100cm_test, 0, 0},	 //	4
				{30, 50, Height_100cm_test, 0, 0},	 //	5
};

void User_Height(void)
{
	//////////////////////////////////////////////////////////////////////
	// 一键起飞/降落例程
	//////////////////////////////////////////////////////////////////////
	// 用静态变量记录一键起飞/降落指令已经执行。
	static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
	static u8 mission_step;
	static float ms = 20;
	static float fc_yaw;

	fc_yaw = fc_att.st_data.yaw_x100 * 0.01f;
	// 判断有遥控信号才执行
	if (rc_in.fail_safe == 0)
	{
		//	通道7复位设置
		if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1300)
		{
			AllInit();
		}
		else
		{
			ch_7_aux3_FLAG = 0;
		}

		// 判断第6通道拨杆位置 1300<CH_6<1700，拨到中间
		if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
		{
			// 还没有执行
			if (one_key_takeoff_f == 0)
			{
				// 标记已经执行
				one_key_takeoff_f =
						// 执行一键起飞
						OneKey_Takeoff(80); // 参数单位：厘米； 0：默认上位机设置的高度。
			}
		}
		else
		{
			// 复位标记，以便再次执行
			one_key_takeoff_f = 0;
		}

    //判断第6通道拨杆位置 800<CH_6<1200
    if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
    {
        if(one_key_land_f==0)
        {
            LX_FC.CtrlMode=0;
            one_key_land_f=1;
            //AngCtrlEnable=0;//角度环控制关闭
            
            ProgramAutoLand=1;//自动降落
        }
    }
    else one_key_land_f=0;

		// 判断第6通道拨杆位置 1700<CH_6<2000
		if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
		{
			// 还没有执行
			if (one_key_mission_f == 0)
			{
				// 标记已经执行
				one_key_mission_f = 1;
				// 开始流程
				mission_step = 1;
			}
		}
		else
		{
			// 复位标记，以便再次执行
			one_key_mission_f = 0;
			LocCtrlEnable = 0;
		}

		if (one_key_mission_f == 1)
		{
			// Timer
			static u16 time_dly_cnt_ms;
		
			switch (mission_step)
			{
				case 0:
				{
					// reset
					time_dly_cnt_ms = 0; // 计时器
				}
				break;
				case 1:
				{
					LX_Change_Mode(2); // 切换到模式2
	
					// wait 1s
					if (time_dly_cnt_ms < 1000)
					{
						time_dly_cnt_ms += ms; // ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += FC_Unlock(); // 解锁
					}
				}
				break;
				case 2:
				{
					// wait 2s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms
					}
					else
					{
						//	控制环开启
						LocCtrlEnable = 1;
						AltCtrlEnable = 1;
						AngCtrlEnable = 1;
	
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 3:
				{
					LX_FC.CtrlMode = 0;
				  // OneKeyTakeOff
				  Exp_Loc_Xcm = 0;
				  Exp_Loc_Ycm = 0;
				  Exp_Alt_Zcm = Height_80cm;
				  Exp_Ang_Deg = 0;
				  mission_step += 1;
				  
				  // 旋转测试
				  //									Exp_Loc_Xcm=0;
				  //									Exp_Loc_Ycm=0;
				  //									Exp_Alt_Zcm=Height_80cm;
				  //									Exp_Ang_Deg=72;
					if (ABS(ano_of.of_alt_cm - Height_100cm_test) < 3)
					{
					// wait 3s
						if (time_dly_cnt_ms < 2000)
						{
							time_dly_cnt_ms += ms;  // ms = 20
						} 
						else 
						{
							time_dly_cnt_ms = 0;
							mission_step += 1;
						}
					}
				}
				break;
				case 4:
				{
//					// 自动降落
//					ProgramAutoLand = 1;
//					AngCtrlEnable = 0;
//					
//					// wait 2s
//					if (time_dly_cnt_ms < 5000)
//						time_dly_cnt_ms += ms; // ms=20;
//					else
//						time_dly_cnt_ms = 0;
//						mission_step += 1;
						if(time_dly_cnt_ms < 2000)
						{
							time_dly_cnt_ms+=ms;//ms=20;
						}
						else
						{
							static s16 LandTime =0;
							LandTime+=20; 
					
							if(LandTime>=10*1000 || ano_of.of_alt_cm<=5)//超时或者高度达到就上锁
							{
									rt_tar.st_data.vel_z=0;
									LandTime=0;
									FC_Lock();
							}
							if(ano_of.of_alt_cm>=100)
									rt_tar.st_data.vel_z=-30;
							else if(ano_of.of_alt_cm>=30)
									rt_tar.st_data.vel_z=-15;
							else 
									rt_tar.st_data.vel_z=-9;
						}
				}
				break;
//				case 5:
//				{
//					if (time_dly_cnt_ms < 2000)
//					{
//						// wait 2s
//						time_dly_cnt_ms += ms; // ms=20; 
//						// check lock
//						if (ProgramAutoLand == 0)
//						{
//							time_dly_cnt_ms = 0;
//							mission_step += 1;
//						}
//					}
//					else
//					{
//						time_dly_cnt_ms = 0;
//						mission_step += 1;
//					}
//				}break;
//				case 6:
//				{
//					if(FC_Lock()==1)//上锁，标志位复位
//						AllInit();
//				}break;
//				
				default:	break;
			}
		}
		else
		{
			mission_step = 0;
		}
	}
}

void UserTask_NewControl(void)
{
	//////////////////////////////////////////////////////////////////////
	// 一键起飞/降落例程
	//////////////////////////////////////////////////////////////////////
	// 用静态变量记录一键起飞/降落指令已经执行。
	static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
	static u8 mission_step;
	static float ms = 20;
	static float fc_yaw;

	fc_yaw = fc_att.st_data.yaw_x100 * 0.01f;
	// 判断有遥控信号才执行
	if (rc_in.fail_safe == 0)
	{
		//	通道7复位设置
		if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1300)
		{
			AllInit();
			ch_7_aux3_FLAG = 1;
		}
		else
		{
			ch_7_aux3_FLAG = 0;
		}

		// 判断第6通道拨杆位置 1300<CH_6<1700，拨到中间
		if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
		{
			// 还没有执行
			if (one_key_takeoff_f == 0)
			{
				// 标记已经执行
				one_key_takeoff_f =
						// 执行一键起飞
						OneKey_Takeoff(80); // 参数单位：厘米； 0：默认上位机设置的高度。
			}
		}
		else
		{
			// 复位标记，以便再次执行
			one_key_takeoff_f = 0;
		}

		//
		// 判断第6通道拨杆位置 800<CH_6<1200
		if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
		{
			// 还没有执行
			if (one_key_land_f == 0)
			{
				// 标记已经执行
				one_key_land_f = 1;
				LX_FC.CtrlMode = 0;	 // 定点控制
				ProgramAutoLand = 1; // 自主降落标志
			}
		}
		else
		{
			// 复位标记，以便再次执行
			one_key_land_f = 0;
		}

		// 判断第6通道拨杆位置 1700<CH_6<2000
		if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
		{
			// 还没有执行
			if (one_key_mission_f == 0)
			{
				// 标记已经执行
				one_key_mission_f = 1;
				// 开始流程
				mission_step = 1;
			}
		}
		else
		{
			// 复位标记，以便再次执行
			one_key_mission_f = 0;
			LocCtrlEnable = 0;
		}

		if (one_key_mission_f == 1)
		{
			// Timer
			static u16 time_dly_cnt_ms;

			switch (mission_step)
			{
				case 0:
				{
					// reset
					time_dly_cnt_ms = 0; // 计时器
				}
				break;
				case 1:
				{
					LX_Change_Mode(2); // 切换到模式2
					// wait 1s
					if (time_dly_cnt_ms < 1000)
					{
						time_dly_cnt_ms += ms; // ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += FC_Unlock(); // 解锁
					}
				}
				break;
				case 2:
				{
					// wait 2s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms
					}
					else
					{
						//	控制环开启
						LocCtrlEnable = 1;
						AltCtrlEnable = 1;
						AngCtrlEnable = 1;
	
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				
				case 3:
				{
					// wait 1.5s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms = 20
					}
					else
					{
						time_dly_cnt_ms = 0;
	
						LX_FC.CtrlMode = 0;
						// OneKeyTakeOff  - 80cm
						Exp_Loc_Xcm = Exp_Point[0].Exp_Pos_X;
						Exp_Loc_Ycm = Exp_Point[0].Exp_Pos_Y;
						Exp_Alt_Zcm = Height_80cm;
						Exp_Ang_Deg = Exp_Point[0].Exp_YAW_Deg;
						mission_step += 1;
					}
				}
				break;

				case 4:
				{
					// wait 1.5s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms = 20
					}
					else
					{
						time_dly_cnt_ms = 0;
	
						LX_FC.CtrlMode = 0;
						// first point
						Exp_Loc_Xcm = Exp_Point[1].Exp_Pos_X;
						Exp_Loc_Ycm = Exp_Point[1].Exp_Pos_Y;
						Exp_Alt_Zcm = Height_80cm;
						Exp_Ang_Deg = Exp_Point[1].Exp_YAW_Deg;
						mission_step += 1;
					}
				}
				break;
				
				case 5:
				{
					// wait 1.5s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms = 20
					}
					else
					{
						time_dly_cnt_ms = 0;
	
						LX_FC.CtrlMode = 0;
						// raise To 100cm
						Exp_Loc_Xcm = Exp_Point[1].Exp_Pos_X;
						Exp_Loc_Ycm = Exp_Point[1].Exp_Pos_Y;
						Exp_Alt_Zcm = Height_100cm_test;
						Exp_Ang_Deg = Exp_Point[1].Exp_YAW_Deg;
						mission_step += 1;
					}
				}
				break;
				
				case 6:
				{
					// wait 1.5s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms = 20
					}
					else
					{
						time_dly_cnt_ms = 0;
	
						LX_FC.CtrlMode = 0;
						// second point
						Exp_Loc_Xcm = Exp_Point[2].Exp_Pos_X;
						Exp_Loc_Ycm = Exp_Point[2].Exp_Pos_Y;
						Exp_Alt_Zcm = Height_100cm_test;
						Exp_Ang_Deg = Exp_Point[2].Exp_YAW_Deg;
						mission_step += 1;
					}
				}
				break;
				
				case 7:
				{
					// wait 1.5s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms = 20
					}
					else
					{
						time_dly_cnt_ms = 0;
	
						LX_FC.CtrlMode = 0;
						// third point
						Exp_Loc_Xcm = Exp_Point[3].Exp_Pos_X;
						Exp_Loc_Ycm = Exp_Point[3].Exp_Pos_Y;
						Exp_Alt_Zcm = Height_100cm_test;
						Exp_Ang_Deg = Exp_Point[3].Exp_YAW_Deg;
						mission_step += 1;
					}
				}
				break;
	
				case 8:
				{
					// wait 2s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms=20;
					}
					else
					{
						// 自动降落
						ProgramAutoLand = 1;
						AngCtrlEnable = 0;
	
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				
				case 9:
				{
					// wait 2s
					if (time_dly_cnt_ms < 2000)
					{
						time_dly_cnt_ms += ms; // ms=20;
					}
					else
					{
						// check lock
						if (ProgramAutoLand == 0)
						{
							time_dly_cnt_ms += ms;
							if (time_dly_cnt_ms > 4000)
							{
								// 上锁，复位所有标志位
								if (FC_Lock() == 1)
									AllInit();
								else
									ch_7_aux3_FLAG = 2; // 检查是否上锁
							}
						}
					}
				}
				break;
				default:	break;
			}
		}
		else
		{
			mission_step = 0;
		}
	}
}

// void UserTask_OneKeyCmd(void)
//{
//     //////////////////////////////////////////////////////////////////////
//     //一键起飞/降落例程
//     //////////////////////////////////////////////////////////////////////
//     //用静态变量记录一键起飞/降落指令已经执行。
//     static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
//     static u8 mission_step;
//		static s16 z_speed_out;
//		static s32 real_X, real_Y;        // 运动距离
//		static float LPFout_x, LPFout_y;  // 位置微分滤波
//		static float ms=20;
//		static float fc_yaw;
//
//		fc_yaw = fc_att.st_data.yaw_x100*0.01f;
//     //判断有遥控信号才执行
//     if (rc_in.fail_safe == 0)
//     {
//         //判断第6通道拨杆位置 1300<CH_6<1700，拨到中间
//         if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
//         {
//             //还没有执行
//             if (one_key_takeoff_f == 0)
//             {
//                 //标记已经执行
//                 one_key_takeoff_f =
//                     //执行一键起飞
//                     OneKey_Takeoff(80); //参数单位：厘米； 0：默认上位机设置的高度。
//             }
//         }
//         else
//         {
//             //复位标记，以便再次执行
//             one_key_takeoff_f = 0;
//         }
//
//         //
//         //判断第6通道拨杆位置 800<CH_6<1200
//         if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
//         {
//             //还没有执行
//             if (one_key_land_f == 0)
//             {
//                 //标记已经执行
//                 one_key_land_f =
//                     //执行一键降落
//                     OneKey_Land();
//             }
//         }
//         else
//         {
//             //复位标记，以便再次执行
//             one_key_land_f = 0;
//         }
//
//         //判断第6通道拨杆位置 1700<CH_6<2000
//         if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
//         {
//             //还没有执行
//             if (one_key_mission_f == 0)
//             {
//                 //标记已经执行
//                 one_key_mission_f = 1;
//                 //开始流程
//                 mission_step = 1;
//             }
//         }
//         else
//         {
//             //复位标记，以便再次执行
//             one_key_mission_f = 0;
//         }
//
//         if (one_key_mission_f == 1)
//         {
//				// Timer
//					static u16 time_dly_cnt_ms;
//					rt_tar.st_data.vel_x = rt_tar.st_data.vel_y = rt_tar.st_data.vel_z = rt_tar.st_data.yaw_dps = 0;
//
//					switch(mission_step)
//					{
//						case 0:
//						{
//							//reset
//							time_dly_cnt_ms = 0;//计时器
//						}
//						break;
//						case 1:
//						{
//							LX_Change_Mode(2);//切换到模式2
//
//							rt_tar.st_data.vel_x = 0;//速度初始化
//							rt_tar.st_data.vel_y = 0;
//							// wait 1s
//							if(time_dly_cnt_ms<1000)
//							{
//								time_dly_cnt_ms+=ms;//ms
//							}
//							else
//							{
//								time_dly_cnt_ms = 0;
//								mission_step += FC_Unlock();//解锁
//								// 	Get initial position | New coordinate(0, 0)
//								start_point_xyz[0] = mid360_DATA.pos_x;
//								start_point_xyz[1] = mid360_DATA.pos_y;
//								// start_point_xyz[0] = UWB_DATA.pos_x;
//								// start_point_xyz[1] = UWB_DATA.pos_y;
//							}
//						}
//						break;
//						case 2:
//						{
//						// wait 2s
//							if(time_dly_cnt_ms<2000)
//							{
//								time_dly_cnt_ms+=ms;//ms
//							}
//							else
//							{
//							// Hight: 120cm
//								mission_step += OneKey_Takeoff(Height_80cm);//起飞
//							}
//						}
//						break;
//
//						case 3:
//						{
//							if(LX_FC.sped_state == 1)/*掉线速度置零*/
//							{
//									// Altitude control
//									rt_tar.st_data.vel_z = height_control(ano_of.of_alt_cm,Height_80cm);
//
//									// Position difference differential speed output
//									real_X = mid360_DATA.pos_x - start_point_xyz[0];
//									real_Y = mid360_DATA.pos_y - start_point_xyz[1];
//
//									User_Loc_Ctrl(real_X,real_Y,expect_point_xy_1[0][0],expect_point_xy_1[0][1]);
//
//									rt_tar.st_data.vel_x = LIMIT(User_LocCtrlOutXcm,-25,25);
//									rt_tar.st_data.vel_y = LIMIT(User_LocCtrlOutYcm,-25,25);
//									// rt_tar.st_data.vel_x = LIMIT((expect_point_xy_1[0][0]-real_X)*0.5, -Vel_LIMIT, Vel_LIMIT);
//									// rt_tar.st_data.vel_y = LIMIT((expect_point_xy_1[0][1]-real_Y)*0.5, -Vel_LIMIT, Vel_LIMIT);
//
//									// S_LPF_1(0.5, (expect_point_xy[0][0] - real_X)*0.5, LPFout_x);
//									// S_LPF_1(0.5, (expect_point_xy[0][1] - real_Y)*0.5, LPFout_y);
//
//									// rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
//									// rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);
//									if(ABS(real_X-expect_point_xy_1[0][0])<5&&ABS(real_Y-expect_point_xy_1[0][1])<5)
//									{
//									// wait 1s
//										if(time_dly_cnt_ms<1000)
//										{
//											time_dly_cnt_ms+=ms;//ms=20;
//										}
//										else
//										{
//											time_dly_cnt_ms = 0;
//											mission_step += 1;
//										}
//									}
//							} else {
//									rt_tar.st_data.yaw_dps = 0;  //航向转动角速度，度每秒，逆时针为正
//									rt_tar.st_data.vel_x = 0;    //头向速度，厘米每秒
//									rt_tar.st_data.vel_y = 0;    //左向速度，厘米每秒
//									rt_tar.st_data.vel_z = 0;	 //天向速度，厘米每秒
//							}
//						}
//						break;
//
//						case 4:
//						{
//								z_speed_out = height_control(ano_of.of_alt_cm, Height_100cm_test);
//								// Altitude control
//								rt_tar.st_data.vel_z = LIMIT(z_speed_out,-20,20);
//								if (ABS(ano_of.of_alt_cm - Height_100cm_test) < 5)
//								{
//								// wait 2s
//									if (time_dly_cnt_ms < 2000)
//									{
//										time_dly_cnt_ms += ms;  // ms = 20
//									}
//									else
//									{
//										time_dly_cnt_ms = 0;
//										mission_step += 1;
//									}
//								}
//						}
//						break;
//
//						case 5:
//						{
//							if(LX_FC.sped_state == 1)/*掉线速度置零*/
//							{
//									// Altitude control
//									rt_tar.st_data.vel_z = height_control(ano_of.of_alt_cm,Height_100cm_test);
//
//									// Position difference differential speed output
//									real_X = mid360_DATA.pos_x - start_point_xyz[0];
//									real_Y = mid360_DATA.pos_y - start_point_xyz[1];
//
//									User_Loc_Ctrl(real_X,real_Y,expect_point_xy_1[1][0],expect_point_xy_1[1][1]);
//
//									rt_tar.st_data.vel_x = LIMIT(User_LocCtrlOutXcm,-25,25);
//									rt_tar.st_data.vel_y = LIMIT(User_LocCtrlOutYcm,-25,25);
//
//									// rt_tar.st_data.vel_x = LIMIT((expect_point_xy_1[1][0]-real_X)*0.5, -Vel_LIMIT, Vel_LIMIT);
//									// rt_tar.st_data.vel_y = LIMIT((expect_point_xy_1[1][1]-real_Y)*0.5, -Vel_LIMIT, Vel_LIMIT);
//
//									// S_LPF_1(0.5, (expect_point_xy[0][0] - real_X)*0.5, LPFout_x);
//									// S_LPF_1(0.5, (expect_point_xy[0][1] - real_Y)*0.5, LPFout_y);
//
//									// rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
//									// rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);
//									if(ABS(real_X-expect_point_xy_1[1][0])<5&&ABS(real_Y-expect_point_xy_1[1][1])<5)
//									{
//									// wait 1s
//										if(time_dly_cnt_ms<1000)
//										{
//											time_dly_cnt_ms+=ms;//ms=20;
//										}
//										else
//										{
//											time_dly_cnt_ms = 0;
//											mission_step += 1;
//										}
//									}
//							} else {
//									rt_tar.st_data.yaw_dps = 0;  //航向转动角速度，度每秒，逆时针为正
//									rt_tar.st_data.vel_x = 0;    //头向速度，厘米每秒
//									rt_tar.st_data.vel_y = 0;    //左向速度，厘米每秒
//									rt_tar.st_data.vel_z = 0;	 //天向速度，厘米每秒
//							}
//						}
//						break;
//
//						case 6:
//						{
//							if(LX_FC.sped_state == 1)/*掉线速度置零*/
//							{
//									// Altitude control
//									rt_tar.st_data.vel_z = height_control(ano_of.of_alt_cm,Height_100cm_test);
//
//									// Position difference differential speed output
//									real_X = mid360_DATA.pos_x - start_point_xyz[0];
//									real_Y = mid360_DATA.pos_y - start_point_xyz[1];
//
//									User_Loc_Ctrl(real_X,real_Y,expect_point_xy_1[2][0],expect_point_xy_1[2][1]);
//
//									rt_tar.st_data.vel_x = LIMIT(User_LocCtrlOutXcm,-25,25);
//									rt_tar.st_data.vel_y = LIMIT(User_LocCtrlOutYcm,-25,25);
//
//									// rt_tar.st_data.vel_x = LIMIT((expect_point_xy_1[2][0]-real_X)*0.5, -Vel_LIMIT, Vel_LIMIT);
//									// rt_tar.st_data.vel_y = LIMIT((expect_point_xy_1[2][1]-real_Y)*0.5, -Vel_LIMIT, Vel_LIMIT);
//
//									// S_LPF_1(0.5, (expect_point_xy[0][0] - real_X)*0.5, LPFout_x);
//									// S_LPF_1(0.5, (expect_point_xy[0][1] - real_Y)*0.5, LPFout_y);
//
//									// rt_tar.st_data.vel_x = LIMIT(alpha*UWB_DATA.vel_x+(1-alpha)*LPFout_x, -Vel_LIMIT, Vel_LIMIT);
//									// rt_tar.st_data.vel_y = LIMIT(alpha*UWB_DATA.vel_y+(1-alpha)*LPFout_y, -Vel_LIMIT, Vel_LIMIT);
//									if(ABS(real_X-expect_point_xy_1[2][0])<5&&ABS(real_Y-expect_point_xy_1[2][1])<5)
//									{
//									// wait 1s
//										if(time_dly_cnt_ms<1000)
//										{
//											time_dly_cnt_ms+=ms;//ms=20;
//										}
//										else
//										{
//											time_dly_cnt_ms = 0;
//											mission_step += 1;
//										}
//									}
//							} else {
//									rt_tar.st_data.yaw_dps = 0;  //航向转动角速度，度每秒，逆时针为正
//									rt_tar.st_data.vel_x = 0;    //头向速度，厘米每秒
//									rt_tar.st_data.vel_y = 0;    //左向速度，厘米每秒
//									rt_tar.st_data.vel_z = 0;	 //天向速度，厘米每秒
//							}
//						}
//						break;
//
//						case 7:
//						{
//						// wait 2s
//							if(time_dly_cnt_ms < 2000)
//							{
//								time_dly_cnt_ms+=ms;//ms=20;
//							}
//							else
//							{
//									static s16 LandTime =0;
//									LandTime+=20;
//
//									if(LandTime>=10*1000 || ano_of.of_alt_cm<=5)//超时或者高度达到就上锁
//									{
//											rt_tar.st_data.vel_z=0;
//											LandTime=0;
//											FC_Lock();
//									}
//									if(ano_of.of_alt_cm>=100)
//											rt_tar.st_data.vel_z=-30;
//									else if(ano_of.of_alt_cm>=30)
//											rt_tar.st_data.vel_z=-15;
//									else
//											rt_tar.st_data.vel_z=-9;
//
//							}
//						}
//						break;
//
//						default: break;
//					}
//				}
//		}
//		else
//		{
//				mission_step = 0;
//		}
// }

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
//						break;
