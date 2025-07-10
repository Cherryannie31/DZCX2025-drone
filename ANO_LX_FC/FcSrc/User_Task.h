#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "user_data.h"
#include "mid360.h"

typedef struct{
	s32 view_vel_x;
	s32 view_vel_y;
} view_vel_data;

void UserTask_OneKeyCmd(void);
extern view_vel_data view;

#endif
