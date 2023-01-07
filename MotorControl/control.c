#include "control.h"
extern Motor motor_right;
extern Motor motor_left;
extern Car car;

/****初始化函数****/
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

/****底层控制函数****/

//PWM输出控制电机转动，并设置电机速度
//rate 取值-100-100，对应电机驱动力矩输出
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

/****数据接收和处理函数****/

//测速函数
float leftV_last=0;
float rightV_last=0;
void Cal_SV(void)
{
	//右轮
	if(motor_right.If_Move==1){
		motor_right.If_Move = 0;
		motor_right.Speed = 10000*d_distance/motor_right.Tim_Delta;
	}
	else if(motor_right.If_Move==2){
		motor_right.If_Move = 0;
		motor_right.Speed = -10000*d_distance/motor_right.Tim_Delta;
	}else{
		motor_right.Speed = 0;
	}
	//左轮
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
	motor_left.Speed = low_pass_filter(motor_left.Speed,leftV_last);
	motor_right.Speed = low_pass_filter(motor_right.Speed,rightV_last);
	leftV_last = motor_left.Speed;
	rightV_last = motor_right.Speed;
	
	car.vecolty = (motor_left.Speed+motor_right.Speed)/2;
}

//一阶低通滤波
#define a 0.8 //滤波系数
float low_pass_filter(float new_value,float last_value)
{
	float flitered = new_value*(1-a) + last_value*a;
	return flitered;
}

/**********************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//输入捕获中断
{
//	uint8_t IsForward_Left = 0,IsForward_Right = 0;
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)//右轮
	{
		motor_right.Tim_This = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
		if(motor_right.Tim_This > motor_right.Tim_Last)
		{
			motor_right.Tim_Delta = motor_right.Tim_This - motor_right.Tim_Last;
		}
		else//一次溢出
		{
			motor_right.Tim_Delta = motor_right.Tim_This + 0xffff - motor_right.Tim_Last;
		}
		motor_right.Tim_Last = motor_right.Tim_This;
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))//正转
		{
			motor_right.If_Move = 1;
			motor_right.Dis += d_distance;
		}
		else//反转
		{
			motor_right.If_Move = 2;
			motor_right.Dis -= d_distance;
		}
	}
	
	
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)//左轮
	{
		//motor_left.If_Move = 1;
		motor_left.Tim_This = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
		if(motor_left.Tim_This > motor_left.Tim_Last)
		{
			motor_left.Tim_Delta = motor_left.Tim_This - motor_left.Tim_Last;
		}
		else//一次溢出
		{
			motor_left.Tim_Delta = motor_left.Tim_This + 0xffff - motor_left.Tim_Last;
		}
		motor_left.Tim_Last = motor_left.Tim_This;
		//正转
		if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14))
		{
			motor_left.If_Move = 1;
			motor_left.Dis += d_distance;
		}
		else//反转
		{
			motor_left.If_Move = 2;
			motor_left.Dis -= d_distance;
		}
	}
	
	
	//求平均得小车行驶位移
	car.Distance = (motor_left.Dis + motor_right.Dis)/2;
}