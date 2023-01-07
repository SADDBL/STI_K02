#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "main.h"
#include "tim.h"
#define circle 21 //�����ܳ���21cm
#define d_distance 0.054 ///0.054,circle/390,һ����������ת���ľ���,0.054cm
#define M_Angle -2.05//1.68//2.900

typedef struct motor{
	float Speed;//m/s
	float Last_Dis;//Motor Distance before
	float Dis;//Motor Distance
	int32_t Tim_This;//��λΪms
	int32_t Tim_Last;
	int32_t Tim_Delta;
	int If_Move;//�ж��Ƿ�ת����1Ϊת����0Ϊ��ת��
	int num;//���ֱ�ţ�1Ϊ��2Ϊ��
}Motor;

typedef struct car
{
	Motor *motor_right_p;
	Motor *motor_left_p;
	float Distance;//���룬��λΪ ����
	float Last_Distance;
	float vecolty;//�ٶȣ���λΪ m/s
	float angle;//�Ƕ�ֵ����ʼֵΪ90��
}Car;


void __Set_PWM_Right(float rate);
void __Set_PWM_Left(float rate);
void carInit(Car *car,Motor *motor_left,Motor *motor_right);
void motorInit(Motor *motor,int num);
void Cal_SV(void);
float low_pass_filter(float new_value,float last_value);

#endif