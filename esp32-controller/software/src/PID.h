#ifndef _USER_PID_H_
#define _USER_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#endif

//PID参数结构体
typedef struct _PID
{
	float kp,ki,kd;
	float error,lastError;
	float integral,maxIntegral;
	float output,maxOutput;
	float deadzone;
	float errLpfRatio;
}PID;

//串级PID参数结构体
typedef struct _CascadePID
{
	PID inner;
	PID outer;
	float output;
}CascadePID;

void PID_Init(PID *pid,float p,float i,float d,float maxSum,float maxOut);
void PID_SingleCalc(PID *pid,float reference,float feedback);
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void PID_Clear(PID *pid);
void PID_SetMaxOutput(PID *pid,float maxOut);
void PID_SetDeadzone(PID *pid,float deadzone);
void PID_SetErrLpfRatio(PID *pid,float ratio);

#ifdef __cplusplus
}
#endif

#endif
