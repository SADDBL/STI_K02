#include "pid.h"


//PID初始化函数
void __InitPID(float kp, float ki, float kd, float outMax, float outMin, PIDstruct *pid)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->p = 0;
	pid->i = 0;
	pid->d = 0;
	pid->de = 0;
	pid->last_de = 0;
	pid->outMax = outMax;
	pid->outMin = outMin;
	pid->output = 0;
}

//PID实现函数
float __RealizePID(PIDstruct *pid, float err)
{
	pid->de = err;
	
	pid->p = pid->kp*pid->de;
	pid->i = pid->i + pid->de;
	pid->d = pid->kd*(pid->de-pid->last_de);
	//积分限幅
	if(pid->i>400){
		pid->i=400;
	}else if(pid->i<-400){
		pid->i=-400;
	}
	pid->output = pid->p + pid->ki*pid->i +pid->d;
	
	if(pid->output > pid->outMax) pid->output = pid->outMax;
	else if(pid->output<pid->outMin) pid->output = pid->outMin;
	
	pid->last_de = pid->de;
	return pid->output;
}

/*****************************************************************/
//电机控制环
void __loop_VPwm(float Vtarget,PIDstruct *PID,Motor motor)
{
	float PWM_Out = __RealizePID(PID, Vtarget-motor.Speed);
	if(motor.num==1){//左轮
		__Set_PWM_Left(PWM_Out);
	}else if(motor.num==2){//右轮
		__Set_PWM_Right(PWM_Out);
	}
}

//直立环和速度环组成并级PID环

//直立环
float __loop_Vertical(float Atarget,PIDstruct *PID,float cur_Angle)
{
	float V_Out = __RealizePID(PID,Atarget -cur_Angle);
	return -V_Out;//输出目标速度
}

//速度环
float __loop_V(float Vtarget, PIDstruct *PID, float cur_V)
{
	float A_Out = __RealizePID(PID,Vtarget - cur_V);
	return A_Out;//输出目标角度
}
