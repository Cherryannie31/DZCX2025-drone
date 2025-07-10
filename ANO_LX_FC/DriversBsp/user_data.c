#include "user_data.h"

uwb_anl_data_1 UWB_DATA;
UWB_POS_VAL pos_base;
UWB_VEL_VAL vel_base;
UWB_QX_VAL qx_base;

// Function to normalize a quaternion
void NormalizeQuaternion(uwb_anl_data_1* QX)
{
	float norm = sqrt((QX->q0)*(QX->q0)+(QX->q1)*(QX->q1)+(QX->q2)*(QX->q2));
	
	if (norm>0.0f)
	{
		QX->q0 /= norm;
		QX->q1 /= norm;
		QX->q2 /= norm;
	}
}

// inverse rotation matrix
void Inverse_Rotation_Matrix(uwb_anl_data_1* QX, float Rotation_Matrix[3][3])
{
	// fill in matrix
	Rotation_Matrix[0][0] = 1 - 2*((QX->q2)*(QX->q2));
	Rotation_Matrix[0][1] = (2*(QX->q1)*(QX->q2));
	Rotation_Matrix[0][2] = 2*(QX->q0)*(QX->q2);
	
	Rotation_Matrix[1][0] = (2*(QX->q1)*(QX->q2));
	Rotation_Matrix[1][1] = 1 - 2*((QX->q1)*(QX->q1));
	Rotation_Matrix[1][2] = -2*(QX->q0)*(QX->q1);
	
	Rotation_Matrix[2][0] = -2*(QX->q0)*(QX->q2);
	Rotation_Matrix[2][1] = 2*(QX->q0)*(QX->q1);
	Rotation_Matrix[2][2] = 1;
}

float limit_0(float n)
{
	
	if ((s32)n >= INT32_MAX | (s32)n <= INT32_MIN)
		return 0;
	else 
		return n;
}

/**********************************  
	 Receive Func:
		- recieve from zigbee
		- unpack data
		- frame: AA FF [......] CD 
***********************************/
UWB_Tag_Raw_Data tag_data;
uint8_t IS_rec = 0;
/**
	Recieve func:
		- data_rec[]
		- data_len
		- data_flag
		- frame: 55 04 00 02 ...
**/
void UWB_ReceiveOneByte(uint8_t data)
{
	static uint8_t rec_buf[150];   // total data_num = 128, minus two frames headers = 126
	static uint8_t flag = 0;
	static uint8_t data_len = 0;
	u8 sum;
	s32 result_pos[3], result_vel[3], result_qx[4], v[3];
	s32 px, py, pz;
	
	switch (flag)
	{
		case 0: 
		{
			if (data == 0x55)		// Frame Header
			{
				flag ++;
				data_len = 0;
			}
		} break;
		case 1:
		{
			if (data == 0x01)		// Function Mark
			{
				flag ++;
			} else 
			{
				flag = 0;
			}
		} break;
		case 2:
		{
			rec_buf[data_len++] = data;
			if (data_len == 126)
			{
//				sum += 0x55;
//				sum += 0x01;
//  			// varify the check sum
//				for(int32_t i=0; i<data_len-1; ++i)
//				{
//					sum += rec_buf[i];
//				}
//				
//				if (sum == rec_buf[data_len-1])
					flag ++;
			}
		} break;
		case 3:
		{  
			for (int i=0; i<3; i++)
			{
				// Obtain pos&vel raw data in the UWB coordinate system.
				result_pos[i] = (rec_buf[i*3+2] << 8 | rec_buf[i*3+3] << 16 |rec_buf[i*3+4] << 24) / 256;
				result_vel[i] = ((rec_buf[i*3+11] << 8 |rec_buf[i*3+12] << 16 | rec_buf[i*3+13] << 24) / 256);
			}
			// cm
			px = result_pos[0] / 10; 
	    py = result_pos[1] / 10;
	    pz = result_pos[2] / 10;
			UWB_DATA.pos_x = result_pos[0] / 10; 
	    UWB_DATA.pos_y = result_pos[1] / 10;
	    UWB_DATA.pos_z = result_pos[2] / 10;
														 
			// qx byte           
			for (uint8_t j=0; j<16; j++)
			{
				// q_x   16 byte
				qx_base.byte[j] = rec_buf[j+86];
			}
			
			UWB_DATA.q0 = qx_base.data[0];
			UWB_DATA.q1 = qx_base.data[1];
			UWB_DATA.q2 = qx_base.data[2];
			UWB_DATA.q3 = qx_base.data[3];
			
			// matrix
			float rotation_matrix[3][3];
			NormalizeQuaternion(&UWB_DATA);
			Inverse_Rotation_Matrix(&UWB_DATA, rotation_matrix);
			
			// rotation value: cm
			float vx =  rotation_matrix[0][0]*(result_vel[0]/100.0f) +
									rotation_matrix[0][1]*(result_vel[1]/100.0f) +
									rotation_matrix[0][2]*(result_vel[2]/100.0f);
									
			float vy =  rotation_matrix[1][0]*(result_vel[0]/100.0f) +
									rotation_matrix[1][1]*(result_vel[1]/100.0f) +
									rotation_matrix[1][2]*(result_vel[2]/100.0f);
			
			float vz =  rotation_matrix[2][0]*(result_vel[0]/100.0f) +
									rotation_matrix[2][1]*(result_vel[1]/100.0f) +
									rotation_matrix[2][2]*(result_vel[2]/100.0f);
			
			if (vx < 1000 || vy < 1000 || vz < 1000)
			{
				UWB_DATA.vel_x = vy;
				UWB_DATA.vel_y = -vx;
				UWB_DATA.vel_z = vz;
			}

			// receive complish   
			UWB_DATA.run_flag = 1;
			flag = 0; 
		} break;
		default: break;
	}
}




void openMV_ReceiveOneByte(uint8_t data)
{
	static uint8_t rec_buf[150];   // total data_num = 128, minus two frames headers = 126
	static uint8_t flag = 0;
	static uint8_t data_len = 0;
	u8 sum;
	s32 result_pos[3], result_vel[3], result_qx[4], v[3];
	s32 px, py, pz;
	
	switch (flag)
	{
		case 0: 
		{
			if (data == 0x55)		// Frame Header
			{
				flag ++;
				data_len = 0;
			}
		} break;
		case 1:
		{
			if (data == 0x01)		// Function Mark
			{
				flag ++;
			} else 
			{
				flag = 0;
			}
		} break;
		case 2:
		{
			rec_buf[data_len++] = data;
			if (data_len == 126)
			{
//				sum += 0x55;
//				sum += 0x01;
//  			// varify the check sum
//				for(int32_t i=0; i<data_len-1; ++i)
//				{
//					sum += rec_buf[i];
//				}
//				
//				if (sum == rec_buf[data_len-1])
					flag ++;
			}
		} break;
		case 3:
		{  
			for (int i=0; i<3; i++)
			{
				// Obtain pos&vel raw data in the UWB coordinate system.
				result_pos[i] = (rec_buf[i*3+2] << 8 | rec_buf[i*3+3] << 16 |rec_buf[i*3+4] << 24) / 256;
				result_vel[i] = ((rec_buf[i*3+11] << 8 |rec_buf[i*3+12] << 16 | rec_buf[i*3+13] << 24) / 256);
			}
			// cm
			px = result_pos[0] / 10; 
	    py = result_pos[1] / 10;
	    pz = result_pos[2] / 10;
			UWB_DATA.pos_x = result_pos[0] / 10; 
	    UWB_DATA.pos_y = result_pos[1] / 10;
	    UWB_DATA.pos_z = result_pos[2] / 10;
														 
			// qx byte           
			for (uint8_t j=0; j<16; j++)
			{
				// q_x   16 byte
				qx_base.byte[j] = rec_buf[j+86];
			}
			
			UWB_DATA.q0 = qx_base.data[0];
			UWB_DATA.q1 = qx_base.data[1];
			UWB_DATA.q2 = qx_base.data[2];
			UWB_DATA.q3 = qx_base.data[3];
			
			// matrix
			float rotation_matrix[3][3];
			NormalizeQuaternion(&UWB_DATA);
			Inverse_Rotation_Matrix(&UWB_DATA, rotation_matrix);
			
			// rotation value: cm
			float vx =  rotation_matrix[0][0]*(result_vel[0]/100.0f) +
									rotation_matrix[0][1]*(result_vel[1]/100.0f) +
									rotation_matrix[0][2]*(result_vel[2]/100.0f);
									
			float vy =  rotation_matrix[1][0]*(result_vel[0]/100.0f) +
									rotation_matrix[1][1]*(result_vel[1]/100.0f) +
									rotation_matrix[1][2]*(result_vel[2]/100.0f);
			
			float vz =  rotation_matrix[2][0]*(result_vel[0]/100.0f) +
									rotation_matrix[2][1]*(result_vel[1]/100.0f) +
									rotation_matrix[2][2]*(result_vel[2]/100.0f);
			
			if (vx < 1000 || vy < 1000 || vz < 1000)
			{
				UWB_DATA.vel_x = vy;
				UWB_DATA.vel_y = -vx;
				UWB_DATA.vel_z = vz;
			}

			// receive complish   
			UWB_DATA.run_flag = 1;
			flag = 0; 
		} break;
		default: break;
	}
}

/**************************************************************************
函数功能：高度环+自动降落
入口参数：feedback_z_com：反馈高度 exp_z_cm：期望高度
功能说明: 输出的数据让全局变量承受：User_AltCtrlOutZcm
返回  值：无
作    者：TQ
**************************************************************************/
s16 height_control(s16 feedback_z_com,s16 exp_z_cm)
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
         /*限速*/
        sped_out_z_cm=LIMIT(sped_out_z_cm,-15,15); 
     }
    // if(AttCtrlEnable==0)
    // {
    //    sped_out_z_cm=0;
		// }
	  // User_AltCtrlOutZcm=sped_out_z_cm;//高度环控制输出
	  return sped_out_z_cm;//高度环控制输出
}

