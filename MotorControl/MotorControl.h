#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "main.h"
#include "tim.h"
#define circle 21 //�����ܳ���21cm
#define d_distance 0.054 // circle/390��390Ϊ��������,һ����������ת���ľ���,0.054cm

typedef struct motor{
	float Speed;//�ٶȣ���λm/s
	float Dis;//Motor Distance
	int32_t Tim_This;//��λΪms����������Ϊ20ms
	int32_t Tim_Last;
	int32_t Tim_Delta;
	int If_Move;//�ж��Ƿ�ת����1Ϊת����0Ϊ��ת��
	int num;//���ֱ�ţ�1Ϊ��2Ϊ��
}Motor;

typedef struct car
{
	Motor *motor_right_p;
	Motor *motor_left_p;
	float Distance;//С����ʻ���룬��λΪ ����
	float Last_Distance;
	float vecolty;//�ٶȣ���λΪ m/s
	float angle;//����Ƕ�ֵ����ʼֵΪ90��
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
