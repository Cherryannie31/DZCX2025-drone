#include "myFilter.h"

/******************************
	Adjust the A value based on actual conditions, 
	  where "Value" represents the effective value, 
		"new_Value" represents the current sampled value, 
		and the program returns the valid actual value.
******************************/
#define Difference 500
#define Minimum -50
#define Maximum 550

FirstOrderLPF LPF_x;
FirstOrderLPF LPF_y;
FirstOrder_LPF LPF_Vel;
complementaryPar v;

s32 safe_float_to_s32 (float val)
{
	static s32 prev_val;
	if (((s32)val)>INT32_MAX || ((s32)val)<INT32_MIN)
	{
		return prev_val;
	}
	
	prev_val = (s32) prev_val;
	return (s32)prev_val;
}

void FirstOrderLPF_Init(FirstOrderLPF* filter, float tf)
{
	filter->prevOut = 0;
	filter->Tf = tf;
}

void FirstOrder_LPF_Init(FirstOrder_LPF* filter, float tf)
{
	filter->prevOut[0] = 0;
	filter->prevOut[1] = 0;
	
	filter->Out[0] = 0;
	filter->Out[1] = 0;
	
	filter->Tf = tf;
}

s32 E_FirstOrderLPF(FirstOrderLPF* filter, s32 x)
{
	static uint32_t prev_t;
  //获取系统当前时间，单位MS
	uint32_t now_t = GetSysRunTimeMs();
	float dt = (now_t - prev_t)*1e-3f;  // seconds
	if (dt < 0.0f) {
		dt = 1e-3f;
	} else if (dt > 0.3f)
	{
		filter->prevOut = x;
		prev_t = now_t;
		return x;
	}
		
	float alpha = filter->Tf / (filter->Tf+dt);
	float output_f = alpha*filter->prevOut + (1.0f-alpha)*x;
	
	// Round it up.
	s32 output = (output_f>=0) ? (s32)(output_f+0.5f) : (s32)(output_f-0.5f);
	
	// upgrade
	filter->prevOut = output;
	
	return output;
}

void _FirstOrderLPF(FirstOrder_LPF* filter, s32 x, s32 y)
{
	float output_f[2];
	static uint32_t prev_t;
  //获取系统当前时间，单位MS
	uint32_t now_t = GetSysRunTimeMs();
	float dt = (now_t - prev_t)*1e-3f;  // seconds
	if (dt < 0.0f) 
		dt = 1e-3f;
	else if (dt > 0.3f)
		filter->prevOut[0] = x;
		filter->prevOut[1] = y;
		prev_t = now_t;
		
	float alpha = filter->Tf / (filter->Tf+dt);
	output_f[0] = alpha*filter->prevOut[0] + (1.0f-alpha)*x;
	output_f[1] = alpha*filter->prevOut[1] + (1.0f-alpha)*y;
	
	filter->Out[0] = (s32)output_f[0];
	filter->Out[1] = (s32)output_f[1];
	
	// upgrade
	filter->prevOut[0] = filter->Out[0];
	filter->prevOut[1] = filter->Out[1];
}

// complementary_filter
float ComplementaryFilter(float a, s32 position_value, s32 sensor_value) {
    return a * (position_value*1.0f) + (1 - a) * (sensor_value*1.0f);
}



