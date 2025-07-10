/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ���������
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "User_Task.h"
#include "mid360.h"
#include "user_control.h"
#include "user_send.h"
//////////////////////////////////////////////////////////////////////
//�û����������
//////////////////////////////////////////////////////////////////////

static void Loop_1000Hz(void) //1msִ��һ��
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2msִ��һ��
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
	//	λ�û�����ģʽ
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
	
	//	�޷�
	OutXcm = LIMIT(OutXcm,-30,30);
	OutYcm = LIMIT(OutYcm,-30,30);
	OutZcm = LIMIT(OutZcm,-15,15);
	OutDeg = LIMIT(OutDeg,-30,30);
	//////////////////////////////////////////////////////////////////////
	
	//	���
	FC_Ctrl(OutXcm,OutYcm,OutZcm,OutDeg);
}

static void Loop_100Hz(void) //10msִ��һ��
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

static void Loop_50Hz(void) //20msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
	User_Height();  // ���+�����������
//	UserTask_OneKeyCmd();
//	UserTask_NewControl();
	Mid360_UpdateCheck(20);
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50msִ��һ��
{
	//////////////////////////////////////////////////////////////////////
	Mid360send_Data();
	Send_To_Analyze();
	//////////////////////////////////////////////////////////////////////
}

static void Loop_2Hz(void) //500msִ��һ��
{
	
}
//////////////////////////////////////////////////////////////////////
//��������ʼ��
//////////////////////////////////////////////////////////////////////
//ϵͳ�������ã�������ִͬ��Ƶ�ʵġ��̡߳�
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
//�������鳤�ȣ��ж��߳�����
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//��ʼ�������
	for (index = 0; index < TASK_NUM; index++)
	{
		//����ÿ���������ʱ������
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//�������Ϊ1��Ҳ����1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//��������ŵ�main������while(1)�У���ͣ�ж��Ƿ����߳�Ӧ��ִ��
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//ѭ���ж������̣߳��Ƿ�Ӧ��ִ��

	for (index = 0; index < TASK_NUM; index++)
	{
		//��ȡϵͳ��ǰʱ�䣬��λMS
		uint32_t tnow = GetSysRunTimeMs();
		//�����жϣ������ǰʱ���ȥ��һ��ִ�е�ʱ�䣬���ڵ��ڸ��̵߳�ִ�����ڣ���ִ���߳�
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//�����̵߳�ִ��ʱ�䣬������һ���ж�
			sched_tasks[index].last_run = tnow;
			//ִ���̺߳�����ʹ�õ��Ǻ���ָ��
			sched_tasks[index].task_func();
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
