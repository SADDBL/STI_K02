#ifndef _PID_H_
#define _PID_H_
#include "control.h"

typedef struct
{
	float kp, ki, kd;
	float p, i ,d;
	float de, last_de;
	float outMax, outMin;
	float output;
}PIDstruct;

void __InitPID(float kp, float ki, float kd, float outMax, float outMin, PIDstruct *pid);
float __RealizePID(PIDstruct *pid, float err);
void __loop_VPwm(float Vtarget,PIDstruct *PID,Motor motor);

float __loop_Vertical(float Atarget,PIDstruct *PID,float cur_Angle);
float __loop_V(float Vtarget, PIDstruct *PID, float cur_V);

#endif