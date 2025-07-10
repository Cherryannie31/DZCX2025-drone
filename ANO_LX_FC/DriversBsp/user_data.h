#ifndef __USER_DATA_H
#define __USER_DATA_H

#include "SysConfig.h"
#include "Ano_Math.h"
#include "ANO_LX.h"
#include "myFilter.h"
#include "math.h"
#include "LX_FC_Fun.h"

/******  analyzed data  ******/
typedef struct
{
	int run_flag;

	s32 pos_x;
	s32 pos_y;
	s32 pos_z;
	
	s32 vel_x;
	s32 vel_y;
	s32 vel_z;
	
	float q0;
	float q1;
	float q2;
	float q3;
} uwb_anl_data_1;     


/******  raw data in union structure  ******/
typedef union 
{
	s32 data[3];
	u8 byte[12];
} UWB_POS_VAL;

typedef union 
{
	s32 data[3];
	u8 byte[12];
} UWB_VEL_VAL;

typedef union 
{
	float data[4];
	u8 byte[16];
} UWB_QX_VAL;


/******  raw data  ******/
typedef struct
{
	uint8_t UWB_id;
	uint8_t UWB_role;
	
	uint8_t pos_val[9]; 
	uint8_t vel_val[9];
	uint8_t q_x[16];
} UWB_Tag_Raw_Data;




extern uwb_anl_data_1 UWB_DATA;
extern UWB_Tag_Raw_Data tag_data;
extern uint8_t IS_rec;

void NormalizeQuaternion(uwb_anl_data_1* QX);
void Inverse_Rotation_Matrix(uwb_anl_data_1* QX, float Rotation_Matrix[3][3]);
void UWB_DataAnl(uint8_t *data);
void UWB_ReceiveOneByte(uint8_t data);
void send_uwb_data (uint8_t *data);
void send_Data(void);

/*******************************  f407
extern uwb_anl_data_1 UWB_DATA;
void GET_UWB_DATA(u8 data);
static void UWB_DataAnl(uint8_t *data);
*************************************/

#endif
