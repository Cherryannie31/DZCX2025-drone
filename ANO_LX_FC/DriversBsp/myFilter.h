#ifndef __MYFILTER__
#define __MYFILTER__

#include "SysConfig.h"
#include "ANO_LX.h"
#include "Ano_Math.h"

typedef struct {
	s32 prevOut;
	float alpha;
} FirstOrderLPF;


typedef struct {
	float v1;
	float v2;
} complementaryPar;


extern FirstOrderLPF LPF_x;
extern FirstOrderLPF LPF_y;
extern complementaryPar v;

s32 safe_float_to_s32 (float val);
void FirstOrderLPF_Init(FirstOrderLPF* filter, float tf);
s32 E_FirstOrderLPF(FirstOrderLPF* filter, s32 x);
float ComplementaryFilter(float a, s32 position_value, s32 sensor_value);

#endif

