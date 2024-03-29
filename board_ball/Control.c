#include "Control.h"
#define max_step_y 320	//36°
#define max_step_x 320	//36°
#define step_mid_y 0
#define step_mid_x 20

stepper motor1;
stepper motor2;

pid pid_controler1;
pid pid_controler2;

int target_step_y = 0;
int target_step_x = 0;

int fabs_int(int val)
{
	if(val<0) return -val;
	else return val;
}

/********** 平板控制函数 **********/
//利用增量式PID控制平板转动指定角度
void pid_dangle(stepper *motor,int v)
{
	int No = motor->No;

	//y轴电机
	if(No==1){
		motor->pid_concroler->target_val = y_target;
		pos_pid_realize(motor->pid_concroler,y_cur);
		target_step_y += motor->pid_concroler->output;
		motor->target_step = target_step_y;
		if(motor->target_step>=max_step_y)
			motor->target_step=max_step_y;
		else if(motor->target_step<=-max_step_y)
			motor->target_step=-max_step_y;
		motor->Anl_v = v;
	}
	else if(No==2){
		motor->pid_concroler->target_val = x_target;
		pos_pid_realize(motor->pid_concroler,x_cur);
		target_step_x += motor->pid_concroler->output;
		motor->target_step = target_step_x;
		if(motor->target_step>=max_step_x+step_mid_x)
			motor->target_step=max_step_x+step_mid_x;
		else if(motor->target_step<=-max_step_x+step_mid_x)
			motor->target_step=-max_step_x+step_mid_x;
		motor->Anl_v = v;
	}
}

//利用位置式PID控制平板转动指定角度
void pid_angle(stepper *motor,int v)
{
	
}

/********** PID底层函数 **********/
/* PID初始化函数 */
void pid_init(pid* pid_controller,float p,float i,float d,PIDOut_Type max,PIDOut_Type min)
{
	pid_controller->kp = p;
	pid_controller->ki = i;
	pid_controller->kd = d;
	pid_controller->cur_val = 0;
	pid_controller->target_val = 0;
	pid_controller->err = 0;
	pid_controller->err_k1 = 0;
	pid_controller->err_k2 = 0;
	pid_controller->output = 0;
	pid_controller->i=0;
	pid_controller->max = max;
	pid_controller->min = min;
}

/* 增量式PID实现函数 */
void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
{
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	//计算de(k)
	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
	PID->err_k1=PID->err;
	PID->err_k2=PID->err_k1;
}

/* 位置式PID实现函数 */
void pid_realize(pid* PID,PIDIn_Type actual_val)
{
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	PID->i+=PID->err;
	//计算de(k)
	PID->output = PID->kp*PID->err + PID->ki*PID->i + PID->kd*(PID->err - PID->err_k1);
	PID->err_k1=PID->err;
	if(PID->output>PID->max) PID->output=PID->max;
	if(PID->output<PID->min) PID->output=PID->min;
}

/********** 步进电机底层函数 **********/
void stepper_init(stepper* motor,uint16_t stp_pin,GPIO_TypeDef *port,uint16_t dir_pin,uint32_t channel,pid* PID,int No)
{
	motor->No=No;
	motor->Channel = channel;
	motor->Dir_pin=dir_pin;
	motor->Stp_pin=stp_pin;
	motor->Dir_Port = port;
	
	motor->Anl_a = 0;
	motor->Anl_v = 300;
	motor->step_record = 0;	//以初始状态为零点
	motor->target_step = 0;
	
	motor->stepper_running = Stop;
	motor->stepper_dir = Stop;
	
	motor->pid_concroler = PID;
}


/* 设定步进电机需要转过的步数 */
void stepper_ctr(stepper* motor)
{
	if(motor->No == 1){
		if(motor->step_record==motor->target_step&&fabs_int(motor->target_step)<fabs_int(target_step_y))
			return;
	}
	else if(motor->No == 2){
		if(motor->step_record==motor->target_step&&fabs_int(motor->target_step)<fabs_int(target_step_x))
			return;
	}
	//反转
	if(motor->target_step<motor->step_record) {	
		motor->stepper_dir = CCW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_SET);	//Dir_pin打高
	}
	//正转
	else if(motor->target_step>motor->step_record){
		motor->stepper_dir = CW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_RESET);	//Dir_pin拉低
	}
	else return;
	//开启输出比较中断
	HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
	__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
}

