/****************PID运算****************/

#include "./PID.h"

//初始化pid参数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->deadzone=0;
	pid->errLpfRatio=1;
}

//单级pid计算
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//若误差在死区内则error直接置0
		pid->error=0;
	else
		pid->error=reference-feedback;
	//低通滤波
	pid->error=pid->error*pid->errLpfRatio+pid->lastError*(1-pid->errLpfRatio);
	//计算微分
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例
	pid->output+=pid->error*pid->kp;
	//计算积分
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
	pid->output+=pid->integral;
	//输出限幅
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//串级pid计算
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PID_SingleCalc(&pid->outer,angleRef,angleFdb);//计算外环(角度环)
	PID_SingleCalc(&pid->inner,pid->outer.output,speedFdb);//计算内环(速度环)
	pid->output=pid->inner.output;
}

//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//重新设定pid输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	pid->maxOutput=maxOut;
}

//设置PID死区
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}

//设置LPF比例
void PID_SetErrLpfRatio(PID *pid,float ratio)
{
	pid->errLpfRatio=ratio;
}
