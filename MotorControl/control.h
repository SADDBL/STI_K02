#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "main.h"
#include "tim.h"
#define circle 21 //轮子周长，21cm
#define d_distance 0.054 ///0.054,circle/390,一个脉冲轮子转动的距离,0.054cm
#define M_Angle -2.05//1.68//2.900

typedef struct motor{
	float Speed;//m/s
	float Last_Dis;//Motor Distance before
	float Dis;//Motor Distance
	int32_t Tim_This;//单位为ms
	int32_t Tim_Last;
	int32_t Tim_Delta;
	int If_Move;//判断是否转动，1为转动，0为不转动
	int num;//车轮编号，1为左，2为右
}Motor;

typedef struct car
{
	Motor *motor_right_p;
	Motor *motor_left_p;
	float Distance;//距离，单位为 厘米
	float Last_Distance;
	float vecolty;//速度，单位为 m/s
	float angle;//角度值，初始值为90°
}Car;


void __Set_PWM_Right(float rate);
void __Set_PWM_Left(float rate);
void carInit(Car *car,Motor *motor_left,Motor *motor_right);
void motorInit(Motor *motor,int num);
void Cal_SV(void);
float low_pass_filter(float new_value,float last_value);

#endif