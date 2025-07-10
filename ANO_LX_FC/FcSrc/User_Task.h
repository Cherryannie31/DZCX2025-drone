#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "user_data.h"
#include "mid360.h"
#include "user_send.h"
#include "user_control.h"

typedef struct{
	// calibration 
	s16 Exp_Pos_X;
	s16 Exp_Pos_Y;
	s16 Exp_Pos_Z;
	
	// yaw_degree
	s16 Exp_YAW_Deg;
	
	// IsArrive
	uint8_t arrive;
	
} Exp_Point_Typedef;

extern u8 ch_7_aux3_FLAG;

void User_Height(void);
void UserTask_NewControl(void);
void UserTask_OneKeyCmd(void);

#endif
