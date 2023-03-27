#include "Control.h"

stepper motor1;
stepper motor2;

pid pid_controler1;
pid pid_controler2;

/********** 平板控制函数 **********/
//控制平板y轴方向倾斜到指定角度
void board_y_angle(float target_ang,int v)
{
	//y轴近似方程：y = 7.3258x - 0.1349
	float stepper_target_ang = 7.3258*target_ang-0.1349;
	stepper_to_angle(&motor1,stepper_target_ang,v);
}

//控制平板y方向改变指定角度
void board_y_dangle(float d_angle,int v)
{
	//dy=7.3258dx
	int pulse_count = (d_angle*7.3258)/MICRO_STEP_ANGLE;
	if(pulse_count+motor1.step_record>=(int)80/MICRO_STEP_ANGLE) pulse_count = (int)(80/MICRO_STEP_ANGLE)-motor1.step_record;
	else if(pulse_count+motor1.step_record<=0) pulse_count = 0-motor1.step_record;
	stepper_ctr(&motor1,pulse_count);
}

//控制平板x轴方向倾斜到指定角度
void board_x_angle(float target_ang,int v)
{
	//x轴近似方程：y=6.9953x+0.7695
	float stepper_target_ang = 6.9953*target_ang+0.7695;
	stepper_to_angle(&motor2,stepper_target_ang,270);
}

//利用增量式PID控制平板转动指定角度
void pid_dangle(stepper *motor,int v)
{
	int No = motor->No;
	//y轴电机
	if(No==1){
		motor->pid_concroler->target_val = y_target;
		pos_pid_realize(motor->pid_concroler,y_cur);
		board_y_dangle(motor->pid_concroler->output,v);
	}
	else if(No==2){
		motor->pid_concroler->target_val = x_target;
		pos_pid_realize(motor->pid_concroler,x_cur);
		board_x_dangle(motor->pid_concroler->output,v);
	}
}

//控制平板x方向改变指定角度
void board_x_dangle(float d_angle,int v)
{
	//dy=6.553dx
	int pulse_count = (d_angle*6.9953)/MICRO_STEP_ANGLE;
	if(pulse_count+motor2.step_record>=85/MICRO_STEP_ANGLE) pulse_count = (int)(85/MICRO_STEP_ANGLE)-motor2.step_record;
	else if(pulse_count+motor2.step_record<=0) pulse_count = 0-motor2.step_record;
	stepper_ctr(&motor2,pulse_count);
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
	pid_controller->output_max = max;
	pid_controller->output_min = min;
}

/* 增量式PID实现函数 */
void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
{
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	
	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
	PID->err_k1=PID->err;
	PID->err_k2=PID->err_k1;
	if(PID->output> PID->output_max)	PID->output = PID->output_max;
	if(PID->output< PID->output_min)	PID->output = PID->output_min;
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
	motor->Anl_v = 270;
	motor->step_record = 400;	//以45°为零点
	
	motor->pulsenum_left = 0;
	motor->stepper_running = Stop;
	motor->stepper_dir = Stop;
	
	motor->pid_concroler = PID;
}


/* 设定步进电机需要转过的步数 */
void stepper_ctr(stepper* motor,int pulse_count)
{
	if(pulse_count==0) return;
	//正转
	if(pulse_count<0) {	
		motor->stepper_dir = CCW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_SET);	//Dir_pin打高
		pulse_count = -pulse_count;
	}
	//反转
	else if(pulse_count >0){
		motor->stepper_dir = CW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_RESET);	//Dir_pin拉低
		
	}
	
	motor->pulsenum_left = pulse_count;
	//开启输出比较中断
	HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
	__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
}


/* 步进电机转动到指定角度 */
void stepper_to_angle(stepper *motor,float target_ang,int v)
{
	int target_pulse;
	//int dir1_count,dir2_count;
	int pulse_count=0;
	if(target_ang>Angle_MAX) target_ang = Angle_MAX;
	else if(target_ang<Angle_MIN) target_ang = Angle_MIN;
	target_pulse = (int)((target_ang+45)/MICRO_STEP_ANGLE);
	
	if(motor->step_record==target_pulse) return;
	pulse_count = target_pulse-motor->step_record;
	
//	if(motor->step_record>target_pulse)
//	{
//		dir1_count = SPR+target_pulse-motor->step_record;	//正转步数
//		dir2_count = motor->step_record - target_ang;	//反转步数
//	}
//	else if(motor->step_record<target_pulse)
//	{
//		dir1_count = target_pulse-motor->step_record;	//正转步数
//		dir2_count = SPR+motor->step_record - target_ang;	//反转步数
//	}
	
	motor->Anl_v = v;
	stepper_ctr(motor,pulse_count);
//	if(dir1_count<dir2_count) stepper_ctr(motor,dir1_count);
//	else stepper_ctr(motor,-dir2_count);
}

