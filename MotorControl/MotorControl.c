#include"MotorControl.h"
extern Motor motor_right;
extern Motor motor_left;
extern Car car;

/****��ʼ������****/

/**
 * @brief  motor�ṹ���ʼ��
 * @param  *motor:
 * @param  num: ���ӱ�ţ���������������
 * @return ��
 */
void motorInit(Motor *motor,int num)
{
	motor->Speed = 0;
	motor->Dis = 0;
	motor->Tim_This = 0;
	motor->Tim_Last = 0;
	motor->Tim_Delta = 0;
	motor->If_Move=0;
	motor->num=num;
}

void carInit(Car *car,Motor *motor_left,Motor *motor_right)
{
	car->Distance = 0;
	car->vecolty = 0;
	car->motor_left_p = motor_left;
	car->motor_right_p = motor_right;
	car->angle = 90;
}

/****����ײ���ƺ���****/

/**
 * @brief  PWM���Ƶ����PWM��ΧΪ-100-100
 * @param  rate:PWMֵ
 * @return ��
 */

/*
 * ���Ե�����
   HAL_GPIO_WritePin()�������������õĵ�ƽ
   ͬʱ��д�ߣ�GPIO_PIN_SET����Ӧ100-/+rate��д�ͣ�GPIO_PIN_RESET����Ӧ-/+rate
 * ��λ���䣺
   ����__HAL_TIM_SET_COMPARE()�еڶ�������
 */
void __Set_PWM_Left(float rate)
{
	if(rate>=0){
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(uint16_t)10*(100-rate));
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
	}else{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(uint16_t)10*(-rate));
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	}
}
void __Set_PWM_Right(float rate)
{
	if(rate>=0)
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(uint16_t)10*(rate));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(uint16_t)(10*(100+rate)));	
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
	}
}

/****���ݽ��պʹ�����****/

//���ٺ���
void Cal_SV(Motor motor_right,Motor motor_left,Car car)
{
	static float leftV_last=0;
  static float rightV_last=0;
	
	//����
	if(motor_right.If_Move==1){//��ת
		motor_right.If_Move = 0;
		motor_right.Speed = 10000*d_distance/motor_right.Tim_Delta;//ϵ��10000���ڵ�λת��
	}
	else if(motor_right.If_Move==2){//��ת
		motor_right.If_Move = 0;
		motor_right.Speed = -10000*d_distance/motor_right.Tim_Delta;
	}else{//��ת,ת��Ϊ0
		motor_right.Speed = 0;
	}
	
	//����
	if(motor_left.If_Move==1){
		motor_left.If_Move = 0;
		motor_left.Speed = 10000*d_distance/motor_left.Tim_Delta;
	}
	else if(motor_left.If_Move==2){
		motor_left.If_Move = 0;
		motor_left.Speed = -10000*d_distance/motor_left.Tim_Delta;
	}else{
		motor_left.Speed = 0;
	}
	
	//�޷��˲�
	if(motor_left.Speed>1.3){
		motor_left.Speed = leftV_last;
	}else if(motor_left.Speed<-1.3){
		motor_left.Speed = leftV_last;
	}
	if(motor_right.Speed>1.3){
		motor_right.Speed = rightV_last;
	}else if(motor_right.Speed<-1.3){
		motor_right.Speed = rightV_last;
	}
	
	//һ���˲�
	motor_left.Speed = first_order_filter(motor_left.Speed,leftV_last);
	motor_right.Speed = first_order_filter(motor_right.Speed,rightV_last);
	
	leftV_last = motor_left.Speed;
	rightV_last = motor_right.Speed;
	
	car.vecolty = (motor_left.Speed+motor_right.Speed)/2;
}

//һ���˲�
#define a 0.8 //�˲�ϵ��
float first_order_filter(float new_value,float last_value)
{
	//a��ȡֵ�������㷨�������ȣ�aԽ���²ɼ���ֵռ��Ȩ��Խ���㷨Խ��������ƽ˳�Բ�
	//�෴��aԽС���²ɼ���ֵռ��Ȩ��ԽС�������Ȳ��ƽ˳�Ժá�
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}

/****PIDʵ�ֺ���****/

/***PID��������***/
//PID��ʼ������
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

/**
 * @brief  PIDʵ�ֺ���
 * @param  *pid��pid�ṹ��ָ��
 * @param  err����ֵ��Ŀ��ֵ-��ǰֵ�����ڸ�����
 * @return pid->output��pid���ֵ
 */
float __RealizePID(PIDstruct *pid, float err)
{
	pid->de = err;
	
	pid->p = pid->kp*pid->de;
	pid->i = pid->i + pid->de;
	pid->d = pid->kd*(pid->de-pid->last_de);
	//�����޷�
	if(pid->i>400){
		pid->i=400;
	}else if(pid->i<-400){
		pid->i=-400;
	}
	pid->output = pid->p + pid->ki*pid->i +pid->d;
	
	//���ֵ�޷�
	if(pid->output > pid->outMax) pid->output = pid->outMax;
	else if(pid->output<pid->outMin) pid->output = pid->outMin;
	
	pid->last_de = pid->de;
	return pid->output;
}

/***PID��***/

//������ƻ�
void __loop_VPwm(float Vtarget,PIDstruct *PID,Motor motor)
{
	float PWM_Out = __RealizePID(PID, Vtarget-motor.Speed);
	if(motor.num==1){//����
		__Set_PWM_Left(PWM_Out);
	}else if(motor.num==2){//����
		__Set_PWM_Right(PWM_Out);
	}
}

//�ٶȿ��ƻ�
float __loop_V(float Vtarget, PIDstruct *PID, float cur_V)
{
	float A_Out = __RealizePID(PID,Vtarget - cur_V);
	return A_Out;//���Ŀ��Ƕ�
}

/**********************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//���벶���ж�
{
	// �ڱ����У���̥תһ�ܲ���390�����壬����һ�����벶���ж�
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)//����
	{
		motor_right.Tim_This = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
		if(motor_right.Tim_This > motor_right.Tim_Last)
		{
			motor_right.Tim_Delta = motor_right.Tim_This - motor_right.Tim_Last;
		}
		else//����һ�������0xffffΪʱ���ۼ����ֵ
		{
			motor_right.Tim_Delta = motor_right.Tim_This + 0xffff - motor_right.Tim_Last;
		}
		motor_right.Tim_Last = motor_right.Tim_This;
		
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))//��ת
		{
			motor_right.If_Move = 1;
			motor_right.Dis += d_distance;
		}
		else//��ת
		{
			motor_right.If_Move = 2;
			motor_right.Dis -= d_distance;
		}
	}
	
	
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)//����
	{
		motor_left.Tim_This = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
		if(motor_left.Tim_This > motor_left.Tim_Last)
		{
			motor_left.Tim_Delta = motor_left.Tim_This - motor_left.Tim_Last;
		}
		else//һ�����
		{
			motor_left.Tim_Delta = motor_left.Tim_This + 0xffff - motor_left.Tim_Last;
		}
		motor_left.Tim_Last = motor_left.Tim_This;
		//��ת
		if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14))
		{
			motor_left.If_Move = 1;
			motor_left.Dis += d_distance;
		}
		else//��ת
		{
			motor_left.If_Move = 2;
			motor_left.Dis -= d_distance;
		}
	}
	
	
	//��ƽ����С����ʻλ��
	car.Distance = (motor_left.Dis + motor_right.Dis)/2;
}