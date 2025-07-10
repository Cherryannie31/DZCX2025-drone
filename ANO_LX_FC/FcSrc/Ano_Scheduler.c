/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "User_Task.h"
#include "mid360.h"
#include "user_control.h"
#include "user_send.h"
//////////////////////////////////////////////////////////////////////
//用户程序调度器
//////////////////////////////////////////////////////////////////////

static void Loop_1000Hz(void) //1ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5ms执行一次
{
	//////////////////////////////////////////////////////////////////////
	//	位置环控制模式
	if	(LX_FC.CtrlMode==0)
	{
		//	PARAM: feedback_value - exp_value
		height_control(ano_of.of_alt_cm,Exp_Alt_Zcm);
		Loc_Ctrl(mid360_DATA.pos_x,mid360_DATA.pos_y,Exp_Loc_Xcm,Exp_Loc_Ycm);
		Ang_Ctrl(mid360_DATA.yaw,Exp_Ang_Deg);
	}
	
	OutXcm = User_LocCtrlOutXcm;
	OutYcm = User_LocCtrlOutYcm;
	OutZcm = User_AltCtrlOutZcm;
	OutDeg = User_AngCtrlOutDeg;
	
	//	限幅
	OutXcm = LIMIT(OutXcm,-30,30);
	OutYcm = LIMIT(OutYcm,-30,30);
	OutZcm = LIMIT(OutZcm,-15,15);
	OutDeg = LIMIT(OutDeg,-30,30);
	//////////////////////////////////////////////////////////////////////
	
	//	输出
	FC_Ctrl(OutXcm,OutYcm,OutZcm,OutDeg);
}

static void Loop_100Hz(void) //10ms执行一次
{
	//////////////////////////////////////////////////////////////////////
//	static s32 expected_point[2] = {150,100};
//	
//	if (UWB_DATA.run_flag == 1)
//	{
//		static s32 start_point[2];
//		static s32 real_point[2];
//		static uint8_t step;
//		
//		switch (step) {
//			case 0:
//			{
//				FirstOrder_LPF_Init(&LPF_Vel,0.1f);
//				start_point[0] = UWB_DATA.pos_x;
//				start_point[1] = UWB_DATA.pos_y;
//				step ++;
//			}break;
//			case 1:
//			{
//				real_point[0] = UWB_DATA.pos_x - start_point[0];
//				real_point[1] = UWB_DATA.pos_y - start_point[1];
//				
//				s32 x = (s32) ((expected_point[0] - real_point[0])*0.5);
//				s32 y = (s32) ((expected_point[1] - real_point[1])*0.5);
//				
//				if (x < 1000 && y < 1000)
//				{
//					LPF_Vel.Out[0] = LIMIT(S_LPF_1(0.5, x, LPF_Vel.Out[0]), -12, 12);
//					LPF_Vel.Out[1] = LIMIT(S_LPF_1(0.5, y, LPF_Vel.Out[1]), -12, 12);
//				}
//				
//				if (ABS(real_point[0] - expected_point[0] < 10) && ABS(real_point[1] - expected_point[1]) < 10)
//				{
//					LPF_Vel.Out[0] = LPF_Vel.Out[1] = 0;
//				}

//			}break;
//			default: break;
//		}
//	}
	//////////////////////////////////////////////////////////////////////
}

static void Loop_50Hz(void) //20ms执行一次
{
	//////////////////////////////////////////////////////////////////////
	User_Height();  // 起飞+自主降落测试
//	UserTask_OneKeyCmd();
//	UserTask_NewControl();
	Mid360_UpdateCheck(20);
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50ms执行一次
{
	//////////////////////////////////////////////////////////////////////
	Mid360send_Data();
	Send_To_Analyze();
	//////////////////////////////////////////////////////////////////////
}

static void Loop_2Hz(void) //500ms执行一次
{
	
}
//////////////////////////////////////////////////////////////////////
//调度器初始化
//////////////////////////////////////////////////////////////////////
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] =
	{
		{Loop_1000Hz, 1000, 0, 0},
		{Loop_500Hz, 500, 0, 0},
		{Loop_200Hz, 200, 0, 0},
		{Loop_100Hz, 100, 0, 0},
		{Loop_50Hz, 50, 0, 0},
		{Loop_20Hz, 20, 0, 0},
		{Loop_2Hz, 2, 0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for (index = 0; index < TASK_NUM; index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行

	for (index = 0; index < TASK_NUM; index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = GetSysRunTimeMs();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
