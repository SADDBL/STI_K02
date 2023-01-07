#include"MotorControl.h"
extern Motor motor_right;
extern Motor motor_left;
extern Car car;

/****初始化函数****/

/**
 * @brief  motor结构体初始化
 * @param  *motor:
 * @param  num: 轮子编号，用于区分左右轮
 * @return 无
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

/****电机底层控制函数****/

/**
 * @brief  PWM控制电机，PWM范围为-100-100
 * @param  rate:PWM值
 * @return 无
 */

/*
 * 极性调整：
   HAL_GPIO_WritePin()函数，调整设置的电平
   同时，写高（GPIO_PIN_SET）对应100-/+rate，写低（GPIO_PIN_RESET）对应-/+rate
 * 单位适配：
   调整__HAL_TIM_SET_COMPARE()中第二个参数
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

/****数据接收和处理函数****/

//测速函数
void Cal_SV(Motor motor_right,Motor motor_left,Car car)
{
	static float leftV_last=0;
  static float rightV_last=0;
	
	//右轮
	if(motor_right.If_Move==1){//正转
		motor_right.If_Move = 0;
		motor_right.Speed = 10000*d_distance/motor_right.Tim_Delta;//系数10000用于单位转换
	}
	else if(motor_right.If_Move==2){//反转
		motor_right.If_Move = 0;
		motor_right.Speed = -10000*d_distance/motor_right.Tim_Delta;
	}else{//不转,转速为0
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
	
	//限幅滤波
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
	
	//一阶滤波
	motor_left.Speed = first_order_filter(motor_left.Speed,leftV_last);
	motor_right.Speed = first_order_filter(motor_right.Speed,rightV_last);
	
	leftV_last = motor_left.Speed;
	rightV_last = motor_right.Speed;
	
	car.vecolty = (motor_left.Speed+motor_right.Speed)/2;
}

//一阶滤波
#define a 0.8 //滤波系数
float first_order_filter(float new_value,float last_value)
{
	//a的取值决定了算法的灵敏度，a越大，新采集的值占的权重越大，算法越灵敏，但平顺性差
	//相反，a越小，新采集的值占的权重越小，灵敏度差，但平顺性好。
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}

/****PID实现函数****/

/***PID基础函数***/
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

/**
 * @brief  PID实现函数
 * @param  *pid：pid结构体指针
 * @param  err：差值，目标值-当前值，属于负反馈
 * @return pid->output：pid输出值
 */
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
	
	//输出值限幅
	if(pid->output > pid->outMax) pid->output = pid->outMax;
	else if(pid->output<pid->outMin) pid->output = pid->outMin;
	
	pid->last_de = pid->de;
	return pid->output;
}

/***PID环***/

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

//速度控制环
float __loop_V(float Vtarget, PIDstruct *PID, float cur_V)
{
	float A_Out = __RealizePID(PID,Vtarget - cur_V);
	return A_Out;//输出目标角度
}

/**********************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//输入捕获中断
{
	// 在本例中，轮胎转一周产生390个脉冲，触发一次输入捕获中断
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)//右轮
	{
		motor_right.Tim_This = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
		if(motor_right.Tim_This > motor_right.Tim_Last)
		{
			motor_right.Tim_Delta = motor_right.Tim_This - motor_right.Tim_Last;
		}
		else//处理一次溢出，0xffff为时间累计最大值
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