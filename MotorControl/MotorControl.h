#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "main.h"
#include "tim.h"
#define circle 21 //轮子周长，21cm
#define d_distance 0.054 // circle/390（390为脉冲数）,一个脉冲轮子转动的距离,0.054cm

typedef struct motor{
	float Speed;//速度，单位m/s
	float Dis;//Motor Distance
	int32_t Tim_This;//单位为ms，控制周期为20ms
	int32_t Tim_Last;
	int32_t Tim_Delta;
	int If_Move;//判断是否转动，1为转动，0为不转动
	int num;//车轮编号，1为左，2为右
}Motor;

typedef struct car
{
	Motor *motor_right_p;
	Motor *motor_left_p;
	float Distance;//小车行驶距离，单位为 厘米
	float Last_Distance;
	float vecolty;//速度，单位为 m/s
	float angle;//航向角度值，初始值为90°
}Car;

typedef struct
{
	float kp, ki, kd;
	float p, i ,d;
	float de, last_de;
	float outMax, outMin;
	float output;
}PIDstruct;

void motorInit(Motor *motor,int num);
void carInit(Car *car,Motor *motor_left,Motor *motor_right);
void __Set_PWM_Left(float rate);
void __Set_PWM_Right(float rate);
float first_order_filter(float new_value,float last_value);
void Cal_SV(Motor motor_right,Motor motor_left,Car car);

#endif
