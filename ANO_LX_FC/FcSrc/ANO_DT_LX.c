#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_led.h"
#include "LX_FC_State.h"
#include "Drv_Uart.h"
#include "user_data.h"
#include "Drv_AnoOf.h"

/*==========================================================================
 * ÃèÊö    £ºÁèÏö·É¿ØÍ¨ĞÅÖ÷³ÌĞò
 * ¸üĞÂÊ±¼ä£º2020-01-22 
 * ×÷Õß		 £ºÄäÃû¿Æ´´-²è²»Ë¼
 * ¹ÙÍø    £ºwww.anotc.com
 * ÌÔ±¦    £ºanotc.taobao.com
 * ¼¼ÊõQÈº £º190169595
 * ÏîÄ¿ºÏ×÷£º18084888982£¬18061373080
============================================================================
 * ÄäÃû¿Æ´´ÍÅ¶Ó¸ĞĞ»´ó¼ÒµÄÖ§³Ö£¬»¶Ó­´ó¼Ò½øÈº»¥Ïà½»Á÷¡¢ÌÖÂÛ¡¢Ñ§Ï°¡£
 * ÈôÄú¾õµÃÄäÃûÓĞ²»ºÃµÄµØ·½£¬»¶Ó­ÄúÅÄ×©ÌáÒâ¼û¡£
 * ÈôÄú¾õµÃÄäÃûºÃ£¬Çë¶à¶à°ïÎÒÃÇÍÆ¼ö£¬Ö§³ÖÎÒÃÇ¡£
 * ÄäÃû¿ªÔ´³ÌĞò´úÂë»¶Ó­ÄúµÄÒıÓÃ¡¢ÑÓÉìºÍÍØÕ¹£¬²»¹ıÔÚÏ£ÍûÄúÔÚÊ¹ÓÃÊ±ÄÜ×¢Ã÷³ö´¦¡£
 * ¾ı×ÓÌ¹µ´µ´£¬Ğ¡ÈË³£ÆİÆİ£¬ÄäÃû¼á¾ö²»»áÇëË®¾ü¡¢ÇëÅç×Ó£¬Ò²´ÓÎ´ÓĞ¹ıÄ¨ºÚÍ¬ĞĞµÄĞĞÎª¡£  
 * ¿ªÔ´²»Ò×£¬Éú»î¸ü²»ÈİÒ×£¬Ï£Íû´ó¼Ò»¥Ïà×ğÖØ¡¢»¥°ï»¥Öú£¬¹²Í¬½ø²½¡£
 * Ö»ÓĞÄúµÄÖ§³Ö£¬ÄäÃû²ÅÄÜ×öµÃ¸üºÃ¡£  
===========================================================================*/

u8 send_buffer[50]; //·¢ËÍÊı¾İ»º´æ
_dt_st dt;

//===================================================================
void ANO_DT_Init(void)
{
	//========¶¨Ê±´¥·¢
	//
	dt.fun[0x0d].D_Addr = 0xff;
	dt.fun[0x0d].fre_ms = 100;	  //´¥·¢·¢ËÍµÄÖÜÆÚ100ms
	dt.fun[0x0d].time_cnt_ms = 1; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//
	dt.fun[0x40].D_Addr = 0xff;
	dt.fun[0x40].fre_ms = 20;	  //´¥·¢·¢ËÍµÄÖÜÆÚ100ms
	dt.fun[0x40].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//========Íâ²¿´¥·¢
	//
	dt.fun[0x30].D_Addr = 0xff;
	dt.fun[0x30].fre_ms = 0;	  //0 ÓÉÍâ²¿´¥·¢
	dt.fun[0x30].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//
	dt.fun[0x33].D_Addr = 0xff;
	dt.fun[0x33].fre_ms = 0;	  //0 ÓÉÍâ²¿´¥·¢
	dt.fun[0x33].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//
	dt.fun[0x34].D_Addr = 0xff;
	dt.fun[0x34].fre_ms = 0;	  //0 ÓÉÍâ²¿´¥·¢
	dt.fun[0x34].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//
	dt.fun[0x41].D_Addr = 0xff;
	dt.fun[0x41].fre_ms = 0;	  //0 ÓÉÍâ²¿´¥·¢
	dt.fun[0x41].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//
	dt.fun[0xe0].D_Addr = 0xff;
	dt.fun[0xe0].fre_ms = 0;	  //0 ÓÉÍâ²¿´¥·¢
	dt.fun[0xe0].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	//
	dt.fun[0xe2].D_Addr = 0xff;
	dt.fun[0xe2].fre_ms = 0;	  //0 ÓÉÍâ²¿´¥·¢
	dt.fun[0xe2].time_cnt_ms = 0; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	
	
	
	dt.fun[0xf1].D_Addr = 0xff;
	dt.fun[0xf1].fre_ms = 100;	  //´¥·¢·¢ËÍµÄÖÜÆÚ100ms
	dt.fun[0xf1].time_cnt_ms = 2; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
	
	dt.fun[0xf2].D_Addr = 0xff;
	dt.fun[0xf2].fre_ms = 20;	  //´¥·¢·¢ËÍµÄÖÜÆÚ100ms
	dt.fun[0xf2].time_cnt_ms = 2; //ÉèÖÃ³õÊ¼ÏàÎ»£¬µ¥Î»1ms
}

//Êı¾İ·¢ËÍ½Ó¿Ú
static void ANO_DT_LX_Send_Data(u8 *dataToSend, u8 length)
{
	//
	UartSendLXIMU(dataToSend, length);
}

//===================================================================
//Êı¾İ½ÓÊÕ³ÌĞò
//===================================================================
static u8 DT_RxBuffer[256], DT_data_cnt = 0;
void ANO_DT_LX_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	//ÅĞ¶ÏÖ¡Í·ÊÇ·ñÂú×ãÄäÃûĞ­ÒéµÄ0xAA
	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		DT_RxBuffer[0] = data;
	}
	//ÅĞ¶ÏÊÇ²»ÊÇ·¢ËÍ¸ø±¾Ä£¿éµÄÊı¾İ»òÕßÊÇ¹ã²¥Êı¾İ
	else if (rxstate == 1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate = 2;
		DT_RxBuffer[1] = data;
	}
	//½ÓÊÕÖ¡CMD×Ö½Ú
	else if (rxstate == 2)
	{
		rxstate = 3;
		DT_RxBuffer[2] = data;
	}
	//½ÓÊÕÊı¾İ³¤¶È×Ö½Ú
	else if (rxstate == 3 && data < 250)
	{
		rxstate = 4;
		DT_RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	//½ÓÊÕÊı¾İÇø
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		DT_RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
	}
	//½ÓÊÕĞ£Ñé×Ö½Ú1
	else if (rxstate == 5)
	{
		rxstate = 6;
		DT_RxBuffer[4 + _data_cnt++] = data;
	}
	//½ÓÊÕĞ£Ñé×Ö½Ú2£¬±íÊ¾Ò»Ö¡Êı¾İ½ÓÊÕÍê±Ï£¬µ÷ÓÃÊı¾İ½âÎöº¯Êı
	else if (rxstate == 6)
	{
		rxstate = 0;
		DT_RxBuffer[4 + _data_cnt] = data;
		DT_data_cnt = _data_cnt + 5;
		//ano_dt_data_ok = 1;
		ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt);
	}
	else
	{
		rxstate = 0;
	}
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anlº¯ÊıÊÇĞ­ÒéÊı¾İ½âÎöº¯Êı£¬º¯Êı²ÎÊıÊÇ·ûºÏĞ­Òé¸ñÊ½µÄÒ»¸öÊı¾İÖ¡£¬¸Ãº¯Êı»áÊ×ÏÈ¶ÔĞ­ÒéÊı¾İ½øĞĞĞ£Ñé
//Ğ£ÑéÍ¨¹ıºó¶ÔÊı¾İ½øĞĞ½âÎö£¬ÊµÏÖÏàÓ¦¹¦ÄÜ
//´Ëº¯Êı¿ÉÒÔ²»ÓÃÓÃ»§×ÔĞĞµ÷ÓÃ£¬ÓÉº¯ÊıANO_Data_Receive_Prepare×Ô¶¯µ÷ÓÃ
static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	//ÅĞ¶ÏÊı¾İ³¤¶ÈÊÇ·ñÕıÈ·
	if (*(data + 3) != (len - 6))
		return;
	//¸ù¾İÊÕµ½µÄÊı¾İ¼ÆËãĞ£Ñé×Ö½Ú1ºÍ2
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	//¼ÆËã³öµÄĞ£Ñé×Ö½ÚºÍÊÕµ½µÄĞ£Ñé×Ö½Ú×ö¶Ô±È£¬ÍêÈ«Ò»ÖÂ´ú±í±¾Ö¡Êı¾İºÏ·¨£¬²»Ò»ÖÂÔòÌø³ö½âÎöº¯Êı
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //ÅĞ¶ÏsumĞ£Ñé
		return;
	//ÔÙ´ÎÅĞ¶ÏÖ¡Í·ÒÔ¼°Ä¿±êµØÖ·ÊÇ·ñºÏ·¨
	if (*(data) != 0xAA || (*(data + 1) != HW_TYPE && *(data + 1) != HW_ALL))
		return;
	//=============================================================================
	//¸ù¾İÖ¡µÄCMD£¬Ò²¾ÍÊÇµÚ3×Ö½Ú£¬½øĞĞ¶ÔÓ¦Êı¾İµÄ½âÎö
	//PWMÊı¾İ
	if (*(data + 2) == 0X20)
	{
		pwm_to_esc.pwm_m1 = *((u16 *)(data + 4));
		pwm_to_esc.pwm_m2 = *((u16 *)(data + 6));
		pwm_to_esc.pwm_m3 = *((u16 *)(data + 8));
		pwm_to_esc.pwm_m4 = *((u16 *)(data + 10));
		pwm_to_esc.pwm_m5 = *((u16 *)(data + 12));
		pwm_to_esc.pwm_m6 = *((u16 *)(data + 14));
		pwm_to_esc.pwm_m7 = *((u16 *)(data + 16));
		pwm_to_esc.pwm_m8 = *((u16 *)(data + 18));
	}
	//ÁèÏöIMU·¢³öµÄRGBµÆ¹âÊı¾İ
	else if (*(data + 2) == 0X0f)
	{
		led.brightness[0] = *(data + 4);
		led.brightness[1] = *(data + 5);
		led.brightness[2] = *(data + 6);
		led.brightness[3] = *(data + 7);
	}
	//ÁèÏö·É¿Øµ±Ç°µÄÔËĞĞ×´Ì¬
	else if (*(data + 2) == 0X06)
	{
		fc_sta.fc_mode_sta = *(data + 4);
		fc_sta.unlock_sta = *(data + 5);
		fc_sta.cmd_fun.CID = *(data + 6);
		fc_sta.cmd_fun.CMD_0 = *(data + 7);
		fc_sta.cmd_fun.CMD_1 = *(data + 8);
	}
	//·ÉĞĞËÙ¶È
	else if (*(data + 2) == 0X07)
	{
		for(u8 i=0;i<6;i++)
		{
			fc_vel.byte_data[i] = *(data + 4 + i);
		}
	}
	//×ËÌ¬½Ç£¨ĞèÒªÔÚÉÏÎ»»úÁèÏöIMU½çÃæÅäÖÃÊä³ö¹¦ÄÜ£©
	else if (*(data + 2) == 0X03)
	{
		for(u8 i=0;i<7;i++)
		{
			fc_att.byte_data[i] = *(data + 4 + i);
		}		
	}
	//×ËÌ¬ËÄÔªÊı
	else if (*(data + 2) == 0X03)
	{
		for(u8 i=0;i<9;i++)
		{
			fc_att_qua.byte_data[i] = *(data + 4 + i);
		}			
	}
	//´«¸ĞÆ÷Êı¾İ
	else if (*(data + 2) == 0X01)
	{
		/*
		acc_x = *((s16 *)(data + 4));
		acc_y = *((s16 *)(data + 6));
		acc_z = *((s16 *)(data + 8));
		gyr_x = *((s16 *)(data + 10));
		gyr_y = *((s16 *)(data + 12));
		gyr_z = *((s16 *)(data + 14));	
		state = *(data + 16);
		*/
	}
	//ÃüÁîE0£¬¾ßÌåÃüÁî¸ñÊ½¼°¹¦ÄÜ£¬²Î¼ûÄäÃûÍ¨ĞÅĞ­ÒéV7°æ
	else if (*(data + 2) == 0XE0)
	{
		//¸ù¾İÃüÁîID£º(*(data + 4)) £¬À´Ö´ĞĞ²»Í¬µÄÃüÁî
		switch (*(data + 4))
		{
		case 0x01:
		{
		}
		break;
		case 0x02:
		{
		}
		break;
		case 0x10:
		{
		}
		break;
		case 0x11:
		{
		}
		break;
		default:
			break;
		}
		//ÊÕµ½ÃüÁîºó£¬ĞèÒª·µ»Ø¶ÔÓ¦µÄÓ¦´ğĞÅÏ¢£¬Ò²¾ÍÊÇCK_Backº¯Êı
		dt.ck_send.ID = *(data + 4);
		dt.ck_send.SC = check_sum1;
		dt.ck_send.AC = check_sum2;
		CK_Back(SWJ_ADDR, &dt.ck_send);
	}
	//ÊÕµ½µÄÊÇck·µ»Ø
	else if (*(data + 2) == 0X00)
	{
		//ÅĞ¶ÏÊÕµ½µÄCKĞÅÏ¢ºÍ·¢ËÍµÄCKĞÅÏ¢ÊÇ·ñÏàµÈ
		if ((dt.ck_back.ID == *(data + 4)) && (dt.ck_back.SC == *(data + 5)) && (dt.ck_back.AC == *(data + 6)))
		{
			//Ğ£Ñé³É¹¦
			dt.wait_ck = 0;
		}
	}
	//¶ÁÈ¡²ÎÊı
	else if (*(data + 2) == 0XE1)
	{
		//»ñÈ¡ĞèÒª¶ÁÈ¡µÄ²ÎÊıµÄid
		u16 _par = *(data + 4) + *(data + 5) * 256;
		dt.par_data.par_id = _par;
		dt.par_data.par_val = 0;
		//·¢ËÍ¸Ã²ÎÊı
		PAR_Back(0xff, &dt.par_data);
	}
	//Ğ´Èë²ÎÊı
	else if (*(data + 2) == 0xE2)
	{
		//Ä¿Ç°ÁèÏö¿ªÔ´MCU²»Éæ¼°²ÎÊıµÄĞ´Èë£¬ÍÆ¼ö´ó¼ÒÖ±½ÓÊ¹ÓÃÔ´Âë·½Ê½µ÷Õû×Ô¼º¶¨ÒåµÄ²ÎÊı£¬¹Ê´Ë´¦Ö»·µ»Ø¶ÔÓ¦µÄCKĞ£ÑéĞÅÏ¢
		//		u16 _par = *(data+4)+*(data+5)*256;
		//		u32 _val = (s32)(((*(data+6))) + ((*(data+7))<<8) + ((*(data+8))<<16) + ((*(data+9))<<24));
		//
		dt.ck_send.ID = *(data + 4);
		dt.ck_send.SC = check_sum1;
		dt.ck_send.AC = check_sum2;
		CK_Back(0xff, &dt.ck_send);
		//¸³Öµ²ÎÊı
		//Parameter_Set(_par,_val);
	}
}

//===================================================================
//Êı¾İ·¢ËÍÊµÏÖ³ÌĞò
//===================================================================
static void Add_Send_Data(u8 frame_num, u8 *_cnt, u8 send_buffer[])
{
	s16 temp_data;
	s32 temp_data_32;
	//¸ù¾İĞèÒª·¢Ë ÍµÄÖ¡ID£¬Ò²¾ÍÊÇframe_num£¬À´Ìî³äÊı¾İ£¬Ìî³äµ½send_bufferÊı×éÄÚ
	switch (frame_num)
	{
	case 0x00: //CHECK·µ»Ø
	{
		send_buffer[(*_cnt)++] = dt.ck_send.ID;
		send_buffer[(*_cnt)++] = dt.ck_send.SC;
		send_buffer[(*_cnt)++] = dt.ck_send.AC;
	}
	break;
	case 0x0d: //µç³ØÊı¾İ
	{
		for (u8 i = 0; i < 4; i++)
		{
			send_buffer[(*_cnt)++] = fc_bat.byte_data[i];
		}
	}
	break;
	case 0x30: //GPSÊı¾İ
	{
		//
		for (u8 i = 0; i < 23; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.fc_gps.byte[i];
		}
	}
	break;
	case 0x33: //Í¨ÓÃËÙ¶È²âÁ¿Êı¾İ
	{
		//
		for (u8 i = 0; i < 6; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.gen_vel.byte[i];
		}
	}
	break;
	case 0x34: //Í¨ÓÃ¾àÀë²âÁ¿Êı¾İ
	{
		//
		for (u8 i = 0; i < 7; i++)
		{
			send_buffer[(*_cnt)++] = ext_sens.gen_dis.byte[i];
		}
	}
	break;
	case 0x40: //Ò£¿ØÊı¾İÖ¡
	{
		for (u8 i = 0; i < 20; i++)
		{
			send_buffer[(*_cnt)++] = rc_in.rc_ch.byte_data[i];
		}
	}
	break;
	case 0x41: //ÊµÊ±¿ØÖÆÊı¾İÖ¡
	{
		for (u8 i = 0; i < 14; i++)
		{
			send_buffer[(*_cnt)++] = rt_tar.byte_data[i];
		}
	}
	break;
	case 0xe0: //CMDÃüÁîÖ¡
	{
		send_buffer[(*_cnt)++] = dt.cmd_send.CID;
		for (u8 i = 0; i < 10; i++)
		{
			send_buffer[(*_cnt)++] = dt.cmd_send.CMD[i];
		}
	}
	break;
	case 0xe2: //PARA·µ»Ø
	{
		temp_data = dt.par_data.par_id;
		send_buffer[(*_cnt)++] = BYTE0(temp_data);
		send_buffer[(*_cnt)++] = BYTE1(temp_data);
		temp_data_32 = dt.par_data.par_val;
		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
	}
	break;
	case 0xf1: //CHECK·µ»Ø
	{
//		send_buffer[(*_cnt)++] = base_link.rasypi_mod;//1
//		
//		//temp_data = (s16)(fc_att.st_data.yaw_x100/100);
//	  temp_data = (s16)(base_link.speed_x_cms);//2*100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data);
//		
//		//temp_data = (s16)yaw_err;
//		temp_data = (s16)(base_link.speed_y_cms);//3*100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data);
//		
//		//temp_data = (s16)(target_yaw);
//		temp_data = (s16)(Mahony.vel_out[0]); //4 *100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data);
//		
//		temp_data = (s16)(Mahony.vel_out[1]);//5 *100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data);
//		
//		temp_data_32 = LPF_Vel.Out[0];//1 * 100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
//		
//		temp_data_32 = LPF_Vel.Out[1];//2 * 100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
		
//		temp_data_32 = v.v1;//3 * 100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
//		
//		temp_data_32 = v.v2;//4 * 100
//		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
//		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
		

		temp_data_32 = (s32) UWB_DATA.vel_x;//5 * 100
		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
		
		temp_data_32 = (s32) UWB_DATA.vel_y;//6 * 100
		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
		
		temp_data_32 = (s32) UWB_DATA.vel_z;//7 * 100
		send_buffer[(*_cnt)++] = BYTE0(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE1(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE2(temp_data_32);
		send_buffer[(*_cnt)++] = BYTE3(temp_data_32);
//		
//		send_buffer[(*_cnt)++] = base_link.QR_code;
	}
	break;		
	case 0xf2: //CHECK·µ»Ø
	{
		temp_data = ano_of.of2_dx_fix; 
		send_buffer[(*_cnt)++] = BYTE0(temp_data);
		send_buffer[(*_cnt)++] = BYTE1(temp_data);
		
		temp_data = ano_of.of2_dy_fix; 
		send_buffer[(*_cnt)++] = BYTE0(temp_data);
		send_buffer[(*_cnt)++] = BYTE1(temp_data);
	}
	break;		
	default:
		break;
	}
}

//===================================================================

static void Frame_Send(u8 frame_num, _dt_frame_st *dt_frame)
{
	u8 _cnt = 0;

	send_buffer[_cnt++] = 0xAA;
	send_buffer[_cnt++] = dt_frame->D_Addr;
	send_buffer[_cnt++] = frame_num;
	send_buffer[_cnt++] = 0;
	//==
	//add_send_data
	Add_Send_Data(frame_num, &_cnt, send_buffer);
	//==
	send_buffer[3] = _cnt - 4;
	//==
	u8 check_sum1 = 0, check_sum2 = 0;
	for (u8 i = 0; i < _cnt; i++)
	{
		check_sum1 += send_buffer[i];
		check_sum2 += check_sum1;
	}
	send_buffer[_cnt++] = check_sum1;
	send_buffer[_cnt++] = check_sum2;
	//
	if (dt.wait_ck != 0 && frame_num == 0xe0)
	{
		dt.ck_back.ID = frame_num;
		dt.ck_back.SC = check_sum1;
		dt.ck_back.AC = check_sum2;
	}
	ANO_DT_LX_Send_Data(send_buffer, _cnt);
}
//===================================================================
//
static void Check_To_Send(u8 frame_num)
{
	//
	if (dt.fun[frame_num].fre_ms)
	{
		//
		if (dt.fun[frame_num].time_cnt_ms < dt.fun[frame_num].fre_ms)
		{
			dt.fun[frame_num].time_cnt_ms++;
		}
		else
		{
			dt.fun[frame_num].time_cnt_ms = 1;
			dt.fun[frame_num].WTS = 1; //±ê¼ÇµÈ´ı·¢ËÍ
		}
	}
	else
	{
		//µÈ´ıÍâ²¿´¥·¢
	}
	//
	if (dt.fun[frame_num].WTS)
	{
		dt.fun[frame_num].WTS = 0;
		//Êµ¼Ê·¢ËÍ
		Frame_Send(frame_num, &dt.fun[frame_num]);
	}
}
//===================================================================

//CMD·¢ËÍ
void CMD_Send(u8 dest_addr, _cmd_st *cmd)
{
	dt.fun[0xe0].D_Addr = dest_addr;
	dt.fun[0xe0].WTS = 1; //±ê¼ÇCMDµÈ´ı·¢ËÍ
	dt.wait_ck = 1;		  //±ê¼ÇµÈ´ıĞ£Ñé
}
//CHECK·µ»Ø
void CK_Back(u8 dest_addr, _ck_st *ck)
{
	dt.fun[0x00].D_Addr = dest_addr;
	dt.fun[0x00].WTS = 1; //±ê¼ÇCMDµÈ´ı·¢ËÍ
}
//PARA·µ»Ø
void PAR_Back(u8 dest_addr, _par_st *par)
{
	dt.fun[0xe2].D_Addr = dest_addr;
	dt.fun[0xe2].WTS = 1; //±ê¼ÇCMDµÈ´ı·¢ËÍ
}

//ÈôÖ¸ÁîÃ»·¢ËÍ³É¹¦£¬»á³ÖĞøÖØĞÂ·¢ËÍ£¬¼ä¸ô50ms¡£
static u8 repeat_cnt;
static inline void CK_Back_Check()
{
	static u8 time_dly;
	if (dt.wait_ck == 1)
	{
		if (time_dly < 50) //50ms
		{
			time_dly++;
		}
		else
		{
			time_dly = 0;
			repeat_cnt++;
			if (repeat_cnt < 5)
			{
				dt.fun[0xe0].WTS = 1; //±ê¼ÇµÈ´ı·¢ËÍ£¬ÖØ·¢
			}
			else
			{
				repeat_cnt = 0;
				dt.wait_ck = 0;
			}
		}
	}
	else
	{
		time_dly = 0;
		repeat_cnt = 0;
	}
}

//1msµ÷ÓÃÒ»´Î£¬ÓÃÓÚÍ¨ĞÅ½»»»Êı¾İ
void ANO_LX_Data_Exchange_Task(float dT_s)
{
	//=====¼ì²âCMDÊÇ·ñ·µ»ØÁËĞ£Ñé
	CK_Back_Check();
	//=====¼ì²âÊÇ·ñ´¥·¢·¢ËÍ
	Check_To_Send(0x30);
	Check_To_Send(0x33);
	Check_To_Send(0x34);
	Check_To_Send(0x40);
	Check_To_Send(0x41);
	Check_To_Send(0xe0);
	Check_To_Send(0xe2);
	Check_To_Send(0x0d);
	
	Check_To_Send(0xf1);
	Check_To_Send(0xf2);
}

//===================================================================
