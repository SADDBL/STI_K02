#include "Control.h"

stepper motor1;
stepper motor2;

pid pid_controler1;
pid pid_controler2;

/********** ƽ����ƺ��� **********/
//����ƽ��y�᷽����б��ָ���Ƕ�
void board_y_angle(float target_ang,int v)
{
	//y����Ʒ��̣�y = 5.0562x + 2.4674
	float stepper_target_ang = 5.0562*target_ang+2.4674;
	stepper_to_angle(&motor1,stepper_target_ang,v);
}

//����ƽ��y����ı�ָ���Ƕ�
void board_y_dangle(float d_angle,int v)
{
	//dy=5.0562dx
	int pulse_count = (d_angle*5.0562)/MICRO_STEP_ANGLE;
	if(pulse_count+motor1.step_record>=(int)70/MICRO_STEP_ANGLE) pulse_count = (int)(70/MICRO_STEP_ANGLE)-motor1.step_record;
	else if(pulse_count+motor1.step_record<=0) pulse_count = 0-motor1.step_record;
	//printf("da=%f,dp=%d,re=%d,ta=%d\r\n",d_angle,pulse_count,motor1.step_record,motor1.step_target);
	stepper_ctr(&motor1,pulse_count);
}

//����ƽ��x�᷽����б��ָ���Ƕ�
void board_x_angle(float target_ang,int v)
{
	//x����Ʒ��̣�y = 5.2909x - 7.2274
	float stepper_target_ang = 5.2909*target_ang+7.2274;
	stepper_to_angle(&motor2,stepper_target_ang,270);
}

//����ƽ��x����ı�ָ���Ƕ�
void board_x_dangle(float d_angle,int v)
{
	//dy=5.2909dx
	int pulse_count = (d_angle*5.2909)/MICRO_STEP_ANGLE;
	if(pulse_count+motor2.step_record>=67/MICRO_STEP_ANGLE) pulse_count = (int)(67/MICRO_STEP_ANGLE)-motor2.step_record;
	else if(pulse_count+motor2.step_record<=0) pulse_count = 0-motor2.step_record;
	stepper_ctr(&motor2,pulse_count);
}

//��������ʽPID����ƽ��ת��ָ���Ƕ�
void pid_dangle(stepper *motor,int v)
{
	int No = motor->No;
	//y����
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

/********** PID�ײ㺯�� **********/
/* PID��ʼ������ */
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

/* ����ʽPIDʵ�ֺ��� */
//void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
//{
//	PID->cur_val = actual_val;
//	PID->err = PID->target_val - PID->cur_val;
//	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
//	PID->err_k1=PID->err;
//	PID->err_k2=PID->err_k1;
//	if(PID->output> PID->output_max)	PID->output = PID->output_max;
//	if(PID->output< PID->output_min)	PID->output = PID->output_min;
//}
void pos_pid_realize(pid* PID,PIDIn_Type actual_val)
{
	PIDIn_Type d_e1,d_e2;
	PID->cur_val = actual_val;
	PID->err = PID->target_val - PID->cur_val;
	//����de(k)
	d_e1 = PID->err - PID->err_k1;
	d_e2 = PID->err_k1 - PID->err_k2;
	if(PID->err>e_max1){
		if(PID->err*d_e1<0&&d_e1*d_e2<0){
			PID->output = PID->k1*PID->kp*PID->err;
		}
	}
	PID->output = PID->kp*(PID->err - PID->err_k1) + PID->ki*PID->err + PID->kd*(PID->err + PID->err_k2 - 2*PID->err_k1);
	PID->err_k1=PID->err;
	PID->err_k2=PID->err_k1;
	if(PID->output> PID->output_max)	PID->output = PID->output_max;
	if(PID->output< PID->output_min)	PID->output = PID->output_min;
}


/********** ��������ײ㺯�� **********/
void stepper_init(stepper* motor,uint16_t stp_pin,GPIO_TypeDef *port,uint16_t dir_pin,uint32_t channel,pid* PID,int No)
{
	motor->No=No;
	motor->Channel = channel;
	motor->Dir_pin=dir_pin;
	motor->Stp_pin=stp_pin;
	motor->Dir_Port = port;
	
	motor->Anl_a = 0;
	motor->Anl_v = 650;
	motor->step_record = 311;	//��34.9875��Ϊ���
	
	motor->pulsenum_left[0] = 0;
	motor->pulsenum_left[1] = 0;
	motor->pulsenum_left[2] = 0;
	motor->stepper_running = Stop;
	motor->stepper_dir = Stop;
	
	motor->pid_concroler = PID;
}


/* �趨���������Ҫת���Ĳ��� */
void stepper_ctr(stepper* motor,int pulse_count)
{
	if(pulse_count==0) return;
	//��ת
	if(pulse_count<0) {	
		motor->stepper_dir = CCW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_SET);	//Dir_pin���
		pulse_count = -pulse_count;
		
	}
	//��ת
	else if(pulse_count>0){
		motor->stepper_dir = CW;
		HAL_GPIO_WritePin(motor->Dir_Port,motor->Dir_pin,GPIO_PIN_RESET);	//Dir_pin����
	}
	
	motor->pulsenum_left[0] = pulse_count;
	//��������Ƚ��ж�
	HAL_TIM_OC_Start_IT(&htim4,motor->Channel);
	__HAL_TIM_CLEAR_IT(&htim4,motor->Channel);
}


/* �������ת����ָ���Ƕ� */
void stepper_to_angle(stepper *motor,float target_ang,int v)
{
	int target_pulse;
	//int dir1_count,dir2_count;
	int pulse_count=0;
	if(target_ang>Angle_MAX) target_ang = Angle_MAX;
	else if(target_ang<Angle_MIN) target_ang = Angle_MIN;
	target_pulse = (int)((target_ang+34.9875)/MICRO_STEP_ANGLE);
	
	if(motor->step_record==target_pulse) return;
	pulse_count = target_pulse-motor->step_record;
//	printf("d = %d = %d - %d\r\n",pulse_count,target_pulse,motor->step_record);
//	if(motor->step_record>target_pulse)
//	{
//		dir1_count = SPR+target_pulse-motor->step_record;	//��ת����
//		dir2_count = motor->step_record - target_ang;	//��ת����
//	}
//	else if(motor->step_record<target_pulse)
//	{
//		dir1_count = target_pulse-motor->step_record;	//��ת����
//		dir2_count = SPR+motor->step_record - target_ang;	//��ת����
//	}
	
	motor->Anl_v = v;
	stepper_ctr(motor,pulse_count);
//	if(dir1_count<dir2_count) stepper_ctr(motor,dir1_count);
//	else stepper_ctr(motor,-dir2_count);
}

