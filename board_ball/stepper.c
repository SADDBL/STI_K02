
#include "stepper.h"
struct Stepper Stepper1;
//逻辑部分
//void			StopSetBit(uint8_t* flag)       {*flag  &= 0xfe ;}								//停止
//void			MoveSetBit(uint8_t* flag)       {*flag  &= 0xfe ; *flag |= 0x01;}//运动							//
//void			StateToAcc(uint8_t* flag)				{*flag	&= 0xf9 ; *flag |= 0x02;}//切换到加速状态
//void			StateToRun(uint8_t* flag)				{*flag	&= 0xf9 ; *flag |= 0x04;}//切换到匀速状态
//void			StateToDec(uint8_t* flag)				{*flag	&= 0xf9 ; *flag |= 0x06;}//切换到减速状态
//void			StateToStp(uint8_t* flag)				{*flag	&= 0xf9 ; *flag |= 0x00;}//切换到停止状态

//  //所有的数量单位都经过了100倍放大 0.01度
//速度与加速度的运算存在一些问题，还需要再测试改进
uint16_t Accel_Max=65535;
uint16_t Velocity_Max=65500;

int  midway=0 ; // 计算所需要的中间量
int i = 0;
//------------------------------------------------全局变量------------------------------------------------------//

const int Freq_t=72000000/72;						//计算预分频之后时基单元的倒数
const uint16_t Tim_Period=65535;				//定时器周期
const uint16_t Tim_Prescaler=71;				//定时器预分频系数
const uint16_t PulseWide_Negetive=15;		//负半周时间
const uint16_t Min_Period=50;						//最短周期
const uint16_t Max_Period=10000;				//最长周期

//------------------------------------------------工具函数------------------------------------------------------//
void StepperInOC(Stepper* stp);

/*******************************************************************************
 * @brief       取绝对值
 * @param       要取绝对值的数
 * @retval      无
 *******************************************************************************/
uint32_t myabsi(int a)
{
	if(a>=0){
		return a;
	}else
	{
		return -a;
	}
}
float myabsf(float a)
{
	if(a>=0){
		return a;
	}else
	{
		return -a;
	}
}
/*******************************************************************************
 * @brief       初始化步进电机
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 *******************************************************************************/
void Init_Stepper(Stepper* stp , GPIO_TypeDef* stpdir_gpio_port , uint16_t stpdir_gpio_pin ,\
									TIM_HandleTypeDef* stps_timer , uint16_t stps_ch , float angpp)
{
	stp->CC_FLAG 				= 0; 
	stp->AcceAng 				=	0 ;
	stp->DirPin	 				=	stpdir_gpio_pin ;
	stp->DirPort 				=	stpdir_gpio_port;
	stp->EnPin					= 0;
	stp->EnPort					=	0;
	stp->motor_state 		= 0;
	stp->period_now 		= 0;
	stp->period_rest 		= 0;
	stp->position_ang 	= 0;
	stp->position_ctnow 	= 0;
	stp->stepangle 			= angpp;
	stp->step_counter 	= 0;
	stp->step_threshold = 0;
	stp->StpChannel 		= stps_ch;
	stp->StpTim 				= stps_timer;
	stp->TagAngV 				= 0;						
//	stp->angv_now 			= 0;
	stp->position_ctnow = 0;
	stp->CCR_ValSetter	= 0;
	
	for(i=0;i<6;i++){
		stp->stepff_memory [i]=0;
	}
	switch(stps_ch)
	{
		case TIM_CHANNEL_1 : {
													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR1));
													}
		case TIM_CHANNEL_2 : {
													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR2));
													}		
		case TIM_CHANNEL_3 : {
													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR3));
													}		
		case TIM_CHANNEL_4 : {
													stp->ChannelCCR = (uint32_t *)(&(stp->StpTim ->Instance->CCR4));
													}		
	}
}

/**
 * @brief       利用计数器计算当前坐标（浮点表示）
 * @param       Stepper* stepper: 步进电机
 * @retval      无
 */
void _Cal_PositionAng(struct Stepper* stepper)
{
	stepper->position_ang = stepper->stepangle * stepper->position_ctnow  ;
}

/*******************************************************************************
 * @brief       计算当前角速度（浮点表示）
 * @param       Stepper* stepper: 步进电机
 * @retval      无
 *******************************************************************************/
float _Cal_AngV(struct Stepper* stepper)
{
	if((IFMOVING(stepper->motor_state))){
		if(IfCW((stepper->motor_state)))//CW
		{
			return (stepper->stepangle * Freq_t)/(stepper->period_now) ; 
		}
		else//CCW
		{
			return	-(stepper->stepangle * Freq_t)/(stepper->period_now) ;
		}
	}else{
		return  0;
	}
}



/*******************************************************************************
 * @brief       关闭步进电机
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 *******************************************************************************/
void Stepper_Stop(struct Stepper* stepper)
{
		HAL_TIM_OC_Stop_IT(stepper->StpTim, stepper->StpChannel); 
		stepper->AcceAng					= 0;
		stepper->motor_state 			= 0;
		stepper->period_now 			= 0;
		stepper->period_rest 			= 0;
		stepper->step_counter 		= 0;	 
}
void	Stepper_Pause(struct Stepper* stepper)
{
	HAL_TIM_OC_Stop_IT(stepper->StpTim, stepper->StpChannel); 
}
/*******************************************************************************
 * @brief       计算电机从初速度加速到目标速度需要的角位移,返回的是脉冲步数
 * @param				struct Stepper* stepper 对应的步进电机
 * @retval      int类型，目标位置
 *******************************************************************************/
uint32_t Calculator_AngleAcc(struct Stepper* stepper , float startv , float tagv )//计算电机从初速度加速到目标速度需要的角位移
{
	static float mid = 0;
	mid = (tagv - startv) ;
	return myabsi(mid*(startv + 0.5*mid))/(stepper->AcceAng * stepper->stepangle );   
	
}
/*
					先固定死速度和加速度好了				
					uint32_t stepstoacc1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );
					uint32_t stepstodec  = Calculator_AngleAcc(stepper , _Cal_AngV(stepper) , stepper->TagAngV );
					Calculator_AngleAcc(stepper , _Cal_AngV(stepper) , stepper->TagAngV );
	*/
/*******************************************************************************
 * @brief       计算电机加速度过程中需要的周期
								第一个脉冲不在这里计算
* @param				struct Stepper* stepper 对应的步进电机
								steps 当前运动的总步数
 * @retval      none
 *******************************************************************************/
void CalCulator_NextCount_Accel(struct Stepper* stepper )//计算电机加速度过程中需要的周期
{
	static uint32_t rest = 0;
	
	rest = ((2 *stepper->period_now)+stepper->period_rest)%(4 * stepper->step_counter  + 1);//计算当前的余数
	
	stepper->period_now=(stepper->period_now - ( 2*stepper->period_now + stepper->period_rest )/( 4*stepper->step_counter  + 1));//计算新的周期，使用的是上一次的余数
	
	stepper->period_rest  =  rest ;  //余数赋值
	
	if(stepper->period_now<=Min_Period){//周期限幅
		
		stepper->period_now=Min_Period;
		
	}
	
}


/*******************************************************************************
 * @brief       计算电机减速度过程中需要的周期
								第一个脉冲不在这里计算
 * @param				struct Stepper* stepper 对应的步进电机
								steps 当前运动的总步数
 * @retval      none
 *******************************************************************************/
void CalCulator_NextCount_Deccel(struct Stepper* stepper )//计算电机减速度过程中需要的周期
{
	static uint32_t rest = 0;
	
	rest = ((2 *stepper->period_now)+stepper->period_rest)%(4 * stepper->step_counter  - 1);//计算当前的余数
	
	stepper->period_now=(stepper->period_now + ( 2*stepper->period_now + stepper->period_rest )/( 4*stepper->step_counter  - 1));//计算新的周期，使用的是上一次的余数
	
	stepper->period_rest  =  rest ;  //余数赋值
		
	if(stepper->period_now>=Max_Period){//周期限幅
		
		stepper->period_now=Max_Period;
		
	}
	
}

/*******************************************************************************
 * @brief       初始化要运动的步进电机运动参数
 * @param       Stepper* stepper 电机结构体
 * @retval      无
 *******************************************************************************/
void StpDistanceSetNonBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv )
{
	if(!(IFMOVING(stepper->motor_state)))  //任务下达时电机静止	
	{
		stepper->AcceAng = accel;
		stepper->TagAngV = tagv;
		if(angdistance>=0)
		{
			DIRTOCW(&stepper->motor_state) //顺时针
		}
		else
		{
			DIRTOCCW(&stepper->motor_state)//逆时针
		}			
		stepper->stepff_memory[0]= Calculator_AngleAcc(stepper , 0 , stepper->TagAngV);	
  	stepper->stepff_memory[1]= myabsi((int)angdistance/(stepper->stepangle));
		if( stepper->stepff_memory[0] < (stepper->stepff_memory[1]/2) )
		{//可以加到最大速度
			stepper->stepff_memory[1]	=	stepper->stepff_memory[1]-2*stepper->stepff_memory[0];
			stepper->stepff_memory[2] = stepper->stepff_memory[0];
			StateToAcc(&stepper->motor_state);
			stepper->motor_state&=(~TYPEFLAG);
			stepper->step_threshold = stepper->stepff_memory[0] ;
		}
		else
	  {//中点减速
			stepper->stepff_memory[0] = stepper->stepff_memory[1]/2 + (stepper->stepff_memory[1])%2 ;		
			stepper->stepff_memory[1] = stepper->stepff_memory[1]/2 ;				
			stepper->step_threshold = stepper->stepff_memory[0] ;
			stepper->motor_state&=(~TYPEFLAG);
			stepper->motor_state|=MIDPOINT; //尖峰运动
			StateToAcc(&stepper->motor_state);			
		
		}
		stepper->period_now = 0.676*Freq_t*sqrt(2*stepper->AcceAng/stepper->TagAngV);
		//stepper->period_buffer = (Freq_t* stepper->stepangle)  / tagv;			
		if(stepper->period_now<=Min_Period){//周期限幅
			stepper->period_now=Min_Period;
		}else if(stepper->period_now>=Max_Period){
			stepper->period_now=Max_Period;
		}
		MoveSetBit(&stepper->motor_state);//置位到运动状态
		if((GETDIRECT(stepper->motor_state))==CW){//运动方向
			HAL_GPIO_WritePin(stepper->DirPort , stepper->DirPin  , GPIO_PIN_SET);			
		}else{
			HAL_GPIO_WritePin(stepper->DirPort , stepper->DirPin  , GPIO_PIN_RESET);			
		}
	}
	else   //任务下达时电机在运动
	{
		if((stepper->AcceAng == accel)&&(stepper->TagAngV ==tagv))//同加速度同期望速度
		{
			Stepper_Pause(stepper);
			if(((IfCW(stepper->motor_state))&&(angdistance>=0))||//新增加的运动和原有的运动同方向
				((!IfCW(stepper->motor_state))&&(angdistance<0)))//考虑在哪里插新值
			{
				uint8_t state=GETRUNSTEP(stepper->motor_state);
				if(state==ACCELING)//还在加速
				{				
					if(GETTYPE(stepper->motor_state)==FULLSTEP)
					{
						stepper->stepff_memory[1] += myabsi(angdistance)/stepper->stepangle ;//直接加角度
					}
					else if(GETTYPE(stepper->motor_state)==MIDPOINT)
					{
						uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;
						uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );
						if( 2*steps1 > (stepper->stepff_memory[0]+stepper->stepff_memory[1]+steps1))
						{//还是加速不到
							stepper->stepff_memory[0]+=(steps0/2+steps0%2);
							stepper->stepff_memory[1]+=(steps0/2);
						}
						else//切换到平顶
						{//足够加速到了
							STATETOFULLSTEP((&stepper ->motor_state))
							stepper->stepff_memory[2]=steps1;//变为完整的加减速
							stepper->stepff_memory[1]=steps0-(steps1-stepper->stepff_memory[0]);
							stepper->stepff_memory[0]=steps1;
						}
						stepper->step_threshold=stepper->stepff_memory[0];//赋新的步骤门限				
					}
				}
				else if(state==RUNNING)//还在匀速
				{		
					stepper->stepff_memory[1] += myabsf(angdistance)/stepper->stepangle ;//直接加角度	
					stepper->step_threshold=stepper->stepff_memory[1];//赋新的步骤门限	
				}
				else if((state==DECELING)||(state==STOPING))//在减速或要停止了
				{
						uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;
						uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );
						uint32_t steps2 = stepper->step_counter+steps0;//
						if( 2*steps1 > steps2)
						{//加速不到
							STATETOMIDPOINT((&stepper ->motor_state));
							stepper->stepff_memory[0]=(steps2/2+steps2%2);
							stepper->stepff_memory[1]+=(steps2/2);
						}
						else//切换到平顶
						{//足够加速到了
							STATETOFULLSTEP((&stepper ->motor_state))
							stepper->stepff_memory[2]=steps1;//变为完整的加减速
							stepper->stepff_memory[1]=steps2-2*steps1+ stepper->step_counter;
							stepper->stepff_memory[0]=steps1;
						}
						stepper->step_threshold=stepper->stepff_memory[0];//赋新的步骤门限						
						StateToAcc((&stepper ->motor_state));//重新进入加速段
				}			
			}
			else//新增加的运动和原有的运动不同方向
			{
				Stepper_Pause(stepper);
				//总步数可能存在多走或少走一步的问题
				DIRCHANGENEEDED((&stepper->motor_state));//需要变向,在当前的减速结束后复位为不需要变向的状态
				StateToDec((&stepper->motor_state));//变为减速状态
				uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;
				uint32_t steps1 = 0;//记录在当前方向减速到0需要的步数，反向运动时补偿
				if((GETRUNSTEP(stepper->motor_state))==RUNNING)//在匀速				
				{
					//stepper->step_threshold=stepper->stepff_memory[0];//赋新的步骤门限	
					steps1=stepper->stepff_memory[0];
					stepper->step_counter = steps1=stepper->stepff_memory[0];
				}	
				else if(((GETRUNSTEP(stepper->motor_state))==ACCELING))//还在加速
				{		
					steps1=stepper->step_counter;
				}
				else if((GETRUNSTEP(stepper->motor_state))==DECELING)//在减速或要停止了			
				{
					steps1=stepper->step_counter;
				}
				steps0+=steps1;//补上减速要补的步数,记做反向运动的总步数

				stepper->stepff_memory[0]= Calculator_AngleAcc(stepper , 0 , stepper->TagAngV);	
				stepper->stepff_memory[1]= steps0;
				if( stepper->stepff_memory[0] < (stepper->stepff_memory[1]/2) )
				{//可以加到最大速度
					stepper->stepff_memory[1]	=	stepper->stepff_memory[1]-2*stepper->stepff_memory[0];
					stepper->stepff_memory[2] = stepper->stepff_memory[0];
					stepper->motor_state&=(~TYPEFLAG);
					stepper->step_threshold = stepper->stepff_memory[0] ;
				}
				else
				{//中点减速
					stepper->stepff_memory[0] = stepper->stepff_memory[1]/2 + (stepper->stepff_memory[1])%2 ;		
					stepper->stepff_memory[1] = stepper->stepff_memory[1]/2 ;				
					stepper->step_threshold = stepper->stepff_memory[0] ;
					stepper->motor_state&=(~TYPEFLAG);
					stepper->motor_state|=MIDPOINT; //尖峰运动							
				}
				stepper->period_buffer = 0.676*Freq_t*sqrt(2*stepper->AcceAng / stepper->TagAngV);			 		
				if(stepper->period_buffer<=Min_Period){//周期限幅
					stepper->period_buffer=Min_Period;
				}else if(stepper->period_buffer>=Max_Period){
					stepper->period_buffer=Max_Period;
				}
				if(stepper->step_counter==0)
				{
					stepper->step_counter=1;
				}
										
			}
		}	
	}	
//读取当前通道的信息,怎么表示

	StepperInOC(stepper);
	//__HAL_TIM_SET_COMPARE(stepper->StpTim , stepper->StpChannel  , PulseWide_Negetive);
	HAL_TIM_OC_Start_IT(stepper->StpTim , stepper->StpChannel);		
}



/*******************************************************************************
 * @brief       初始化要运动的步进电机运动参数
 * @param       Stepper* stepper 电机结构体
 * @retval      无
*******************************************************************************/
void StpDistanceSetBlocking( struct Stepper* stepper , float angdistance , float accel , float tagv )
{
	if(!(IFMOVING(stepper->motor_state)))  //任务下达时电机静止	
	{
		stepper->AcceAng = accel;
		stepper->TagAngV = tagv;
		if(angdistance>=0)
		{
			DIRTOCW(&stepper->motor_state) //顺时针
		}
		else
		{
			DIRTOCCW(&stepper->motor_state)//逆时针
		}			
		stepper->stepff_memory[0]= Calculator_AngleAcc(stepper , 0 , stepper->TagAngV);	
  	stepper->stepff_memory[1]= myabsi((int)angdistance/(stepper->stepangle));
		if( stepper->stepff_memory[0] < (stepper->stepff_memory[1]/2) )
		{//可以加到最大速度
			stepper->stepff_memory[1]	=	stepper->stepff_memory[1]-2*stepper->stepff_memory[0];
			stepper->stepff_memory[2] = stepper->stepff_memory[0];
			StateToAcc(&stepper->motor_state);
			stepper->motor_state&=(~TYPEFLAG);
			stepper->step_threshold = stepper->stepff_memory[0] ;
		}
		else
	  {//中点减速
			stepper->stepff_memory[0] = stepper->stepff_memory[1]/2 + (stepper->stepff_memory[1])%2 ;		
			stepper->stepff_memory[1] = stepper->stepff_memory[1]/2 ;				
			stepper->step_threshold = stepper->stepff_memory[0] ;
			stepper->motor_state&=(~TYPEFLAG);
			stepper->motor_state|=MIDPOINT; //尖峰运动
			StateToAcc(&stepper->motor_state);			
		
		}
		stepper->period_now = 0.676*Freq_t*sqrt(2*stepper->AcceAng/stepper->TagAngV);
		//stepper->period_buffer = (Freq_t* stepper->stepangle)  / tagv;			
		if(stepper->period_now<=Min_Period){//周期限幅
			stepper->period_now=Min_Period;
		}else if(stepper->period_now>=Max_Period){
			stepper->period_now=Max_Period;
		}
		MoveSetBit(&stepper->motor_state);//置位到运动状态
		if((GETDIRECT(stepper->motor_state))==CW){//运动方向
			HAL_GPIO_WritePin(stepper->DirPort , stepper->DirPin  , GPIO_PIN_SET);			
		}else{
			HAL_GPIO_WritePin(stepper->DirPort , stepper->DirPin  , GPIO_PIN_RESET);			
		}
	}
	else   //任务下达时电机在运动
	{
		if((stepper->AcceAng == accel)&&(stepper->TagAngV ==tagv))//同加速度同期望速度
		{
			Stepper_Pause(stepper);//避免状态机异常变动
			if(((IfCW(stepper->motor_state))&&(angdistance>=0))||//新增加的运动和原有的运动同方向
				((!IfCW(stepper->motor_state))&&(angdistance<0)))//考虑在哪里插新值
			{
				uint8_t state=GETRUNSTEP(stepper->motor_state);//获取运动的进度
				if(state==ACCELING)//还在加速
				{
					uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;//新增加的步数
					uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );//完全加速需要的步数
					uint32_t steps2 = steps0+stepper->step_counter;//本次运动的步数
					if(steps2>=(2*steps1))
					{
						STATETOFULLSTEP((&stepper ->motor_state));
						stepper->stepff_memory[0] = steps1 ;
						stepper->stepff_memory[1] = steps2-2*steps1;//steps0-steps1-(steps1-stepper->step_counter) ;//直接加角度
						stepper->stepff_memory[2] = steps1 ;
						stepper->step_threshold=stepper->stepff_memory[0];//赋新的步骤门限	
					}
					else if( steps0 <= steps1)//必须要减速了
					{
						stepper->step_counter=steps0;//新的运动将作为减速
						StateToDec((&stepper ->motor_state));
					}
					else{
						STATETOMIDPOINT((&stepper ->motor_state));
						stepper->stepff_memory[0]=steps2/2 + steps2%2;
						stepper->stepff_memory[1]=steps2/2 ;														
						stepper->step_threshold  =stepper->stepff_memory[0];//赋新的步骤门限	
					}
				}
				else if(state==RUNNING)//还在匀速
				{		
					uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;//新增加的步数
					uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );//完全加速需要的步数
					if(steps0>=steps1)
					{
//						STATETOFULLSTEP((&stepper ->motor_state));
//						stepper->stepff_memory[0] = steps1 ;
//						stepper->stepff_memory[1] = steps0;//steps0-steps1-(steps1-stepper->step_counter) ;//直接加角度
//						stepper->stepff_memory[2] = steps1 ;
						stepper->step_threshold=steps0-steps1 ;//赋新的步骤门限	
					}
					else 
					{
						stepper->step_counter=steps0;
						StateToDec((&stepper ->motor_state));
					}
				}
				else if((state==DECELING)||(state==STOPING))//在减速或要停止了
				{
						uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;//新增加的步数
						uint32_t steps1 = Calculator_AngleAcc(stepper , 0 , stepper->TagAngV );//完全加速需要的步数
						uint32_t steps2 = steps0+stepper->step_counter;//本次运动的步数 stepper->step_counter作为新步数
						if(steps0 <= stepper->step_counter)//必须要减速了
						{
								stepper->step_counter=steps0;					
						}
						else if(steps2>=(2*steps1))
						{
							STATETOFULLSTEP((&stepper ->motor_state));
							StateToAcc((&stepper ->motor_state));
							stepper->stepff_memory[0] = steps1 ;
							stepper->stepff_memory[1] = steps2-2*steps1;//steps0-steps1-(steps1-stepper->step_counter) ;//直接加角度
							stepper->stepff_memory[2] = steps1 ;
							stepper->step_threshold=stepper->stepff_memory[0];//赋新的步骤门限	
							if(stepper->step_threshold==stepper->step_counter){
								StateToRun((&stepper ->motor_state));
								stepper->step_counter 	= 0 ;
								stepper->step_threshold=stepper->stepff_memory[1];
							}
						}
						else{
							STATETOMIDPOINT((&stepper ->motor_state));
							StateToAcc((&stepper ->motor_state));
							stepper->stepff_memory[0]=steps2/2 + steps2%2;
							stepper->stepff_memory[1]=steps2/2 ;														
							stepper->step_threshold  =stepper->stepff_memory[0];//赋新的步骤门限	
						}										
				}			
			}
			else//新增加的运动和原有的运动不同方向
			{
				Stepper_Pause(stepper);
				//总步数可能存在多走或少走一步的问题
				DIRCHANGENEEDED((&stepper->motor_state));//需要变向,在当前的减速结束后复位为不需要变向的状态
				StateToDec((&stepper->motor_state));//变为减速状态
				uint32_t steps0 = myabsf(angdistance)/stepper->stepangle ;
				uint32_t steps1 = 0;//记录在当前方向减速到0需要的步数，反向运动时补偿
				if((GETRUNSTEP(stepper->motor_state))==RUNNING)//在匀速				
				{
					//stepper->step_threshold=stepper->stepff_memory[0];//赋新的步骤门限	
					steps1=stepper->stepff_memory[0];
					stepper->step_counter = steps1=stepper->stepff_memory[0];
				}	
				else if(((GETRUNSTEP(stepper->motor_state))==ACCELING))//还在加速
				{		
					steps1=stepper->step_counter;
				}
				else if((GETRUNSTEP(stepper->motor_state))==DECELING)//在减速或要停止了			
				{
					steps1=stepper->step_counter;
				}
				steps0+=steps1;//补上减速要补的步数,记做反向运动的总步数

				stepper->stepff_memory[0]= Calculator_AngleAcc(stepper , 0 , stepper->TagAngV);	
				stepper->stepff_memory[1]= steps0;
				if( stepper->stepff_memory[0] < (stepper->stepff_memory[1]/2) )
				{//可以加到最大速度
					stepper->stepff_memory[1]	=	stepper->stepff_memory[1]-2*stepper->stepff_memory[0];
					stepper->stepff_memory[2] = stepper->stepff_memory[0];
					stepper->motor_state&=(~TYPEFLAG);
					stepper->step_threshold = stepper->stepff_memory[0] ;
				}
				else
				{//中点减速
					stepper->stepff_memory[0] = stepper->stepff_memory[1]/2 + (stepper->stepff_memory[1])%2 ;		
					stepper->stepff_memory[1] = stepper->stepff_memory[1]/2 ;				
					stepper->step_threshold = stepper->stepff_memory[0] ;
					stepper->motor_state&=(~TYPEFLAG);
					stepper->motor_state|=MIDPOINT; //尖峰运动							
				}
				stepper->period_buffer = 0.676*Freq_t*sqrt(2*stepper->AcceAng / stepper->TagAngV);			 		
				if(stepper->period_buffer<=Min_Period){//周期限幅
					stepper->period_buffer=Min_Period;
				}else if(stepper->period_buffer>=Max_Period){
					stepper->period_buffer=Max_Period;
				}
				if(stepper->step_counter==0)
				{
					stepper->step_counter=1;
				}
										
			}
		}	
	}	
//读取当前通道的信息,怎么表示

	StepperInOC(stepper);
	//__HAL_TIM_SET_COMPARE(stepper->StpTim , stepper->StpChannel  , PulseWide_Negetive);
	HAL_TIM_OC_Start_IT(stepper->StpTim , stepper->StpChannel);		
}











/*******************************************************************************
 * @brief       在定时器中断中修改参数
								在运动步骤状态机中查询运动类型
								在状态跳变时触发
								只负责修改状态
 * @param       Stepper* stepper 电机结构体
 * @retval      无
 *******************************************************************************/
void Stepper_Handler(struct Stepper* stepper)
{

	switch((GETRUNSTEP(stepper->motor_state))){
		case ACCELING ://加速段
						{ 
							stepper->step_counter ++ ; 								
							if(GETDIRECT(stepper->motor_state)){
								stepper->position_ctnow +=1;			
							}else{
								stepper->position_ctnow -=1;
							}
							if(stepper->step_counter < stepper->step_threshold)
							{
								CalCulator_NextCount_Accel(stepper);	
							}
							else if (stepper->step_counter >= stepper->step_threshold)
							{//状态机即将结束加速
								CalCulator_NextCount_Accel(stepper);
								if((GETTYPE(stepper->motor_state))==FULLSTEP)
								{//平顶							
									StateToRun(&stepper->motor_state);
									//stepper->period_now = stepper->period_buffer ;
									stepper->step_threshold = stepper->stepff_memory [1] ;	
									stepper->step_counter 	= 0 ;				
								}
								else if((GETTYPE(stepper->motor_state))==MIDPOINT)
								{//尖顶
									StateToDec(&stepper->motor_state);//状态机到减速
									stepper->step_counter 	= stepper->stepff_memory [1] ;
								}
							}
							break;
						}
		case RUNNING ://匀速段
						{
							stepper->step_counter ++ ; 								
							if(GETDIRECT(stepper->motor_state)){
								stepper->position_ctnow +=1;			
							}else{
								stepper->position_ctnow -=1;
							}
							if(stepper->step_counter==stepper->step_threshold){//状态机即将结束匀速
								if((GETTYPE(stepper->motor_state))==FULLSTEP)
								{//平顶或尖顶
									StateToDec(&stepper->motor_state);//状态机到减速
									stepper->step_counter = stepper->stepff_memory [2] ;
								}								
							}	
							break;
						}
		case DECELING ://减速段
						{
								
							if(GETDIRECT(stepper->motor_state)){
								stepper->position_ctnow +=1;			
							}else{
								stepper->position_ctnow -=1;
							}
							CalCulator_NextCount_Deccel(stepper);
							if(stepper->step_counter==1){
								if((GETTYPE(stepper->motor_state))==FULLSTEP||(GETTYPE(stepper->motor_state))==MIDPOINT)
								{//平顶或尖顶
									StateToStp(&stepper->motor_state);//状态机到停止	
								}									
							}							
							stepper->step_counter -- ; 
							break;
						}
		case STOPING : //停止了
						{
							if(GETDIRECT(stepper->motor_state)){
								stepper->position_ctnow +=1;			
							}else{
								stepper->position_ctnow -=1;
							}
							if(IFDIRCHANGE(stepper->motor_state))//要变向，从头开始
							{
								DIRCHANGECANCEL((&stepper->motor_state))
								StateToAcc((&stepper->motor_state))//加速
							  stepper->step_threshold = stepper->stepff_memory [0] ;
								HAL_GPIO_TogglePin(stepper->DirPort , stepper->DirPin );
								stepper->period_now=stepper->period_buffer; 
							}
							else
							{
								Stepper_Stop(stepper);								
							}
							break;
						}
		default : { break; }
	}
	
}

/*******************************************************************************
 * @brief       在定时器中断中修改参数
								在运动步骤状态机中查询运动类型
								在状态跳变时触发
								只负责修改状态
 * @param       Stepper* stepper 电机结构体
 * @retval      无
 *******************************************************************************/
/*
//void StepperInOC(Stepper* stp)
//{
//		if (stp->CC_FLAG){
//			Stepper_Handler(stp);
//			stp->CCR_ValSetter=(Tim_Period-stp->period_now + PulseWide_Negetive);
//			if(htim4.Instance->CCR1 <stp->CCR_ValSetter ) {
//				htim4.Instance->CCR1+=stp->period_now - PulseWide_Negetive;
//			}else{
//				htim4.Instance->CCR1=stp->period_now-stp->CCR_ValSetter;
//			}
//		}
//		else {
//			if(htim4.Instance->CCR1 <stp->CCR_ValSetter ) {
//				htim4.Instance->CCR1+=PulseWide_Negetive;
//			}else{
//				htim4.Instance->CCR1=PulseWide_Negetive-stp->CCR_ValSetter;
//			}	
//		}
//		stp->CC_FLAG = !stp->CC_FLAG;		
//}

//void StepperInOC(Stepper* stp)
//{
//		if (stp->CC_FLAG){
//			Stepper_Handler(stp);
//			stp->CCR_ValSetter=(Tim_Period-stp->period_now + PulseWide_Negetive);
//			if(htim4.Instance->CCR3 <stp->CCR_ValSetter ) {
//				htim4.Instance->CCR3+=stp->period_now - PulseWide_Negetive;
//			}else{
//				htim4.Instance->CCR3=stp->period_now-stp->CCR_ValSetter;
//			}
//		}
//		else {
//			if(htim4.Instance->CCR3 <stp->CCR_ValSetter ) {
//				htim4.Instance->CCR3+=PulseWide_Negetive;
//			}else{
//				htim4.Instance->CCR3=PulseWide_Negetive-stp->CCR_ValSetter;
//			}	
//		}
//		stp->CC_FLAG = !stp->CC_FLAG;		
//}


//void StepperInOC(Stepper* stp)
//{
//		if (stp->CC_FLAG){
//			Stepper_Handler(stp);
//			stp->CCR_ValSetter=(Tim_Period-stp->period_now + PulseWide_Negetive);
//			if((*(stp->ChannelCCR)) < stp->CCR_ValSetter ) {
//				(*(stp->ChannelCCR))+=(stp->period_now - PulseWide_Negetive);
//			}else{
//				(*(stp->ChannelCCR))=(stp->period_now-stp->CCR_ValSetter);
//			}
//		}
//		else {
//			if((*(stp->ChannelCCR)) <stp->CCR_ValSetter ) {
//				(*(stp->ChannelCCR))+=PulseWide_Negetive;
//			}else{
//				(*(stp->ChannelCCR))=(PulseWide_Negetive-stp->CCR_ValSetter);
//			}	
//		}
//		stp->CC_FLAG = !stp->CC_FLAG;		
//}
*/

void StepperInOC(Stepper* stp)
{
		uint16_t Comparenow=__HAL_TIM_GET_COMPARE(stp->StpTim , stp->StpChannel);
	 if (stp->CC_FLAG){
			Stepper_Handler(stp);
			stp->CCR_ValSetter=(Tim_Period-stp->period_now+PulseWide_Negetive);
			if(Comparenow < stp->CCR_ValSetter ) {
				 __HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel , Comparenow+(stp->period_now-PulseWide_Negetive));
			}else{
				__HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel ,(stp->period_now-stp->CCR_ValSetter));
			}
		}
		else {
			if((*(stp->ChannelCCR)) <stp->CCR_ValSetter ) {
			 __HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel ,(Comparenow+PulseWide_Negetive));
			}else{
				__HAL_TIM_SET_COMPARE(stp->StpTim , stp->StpChannel , (PulseWide_Negetive-stp->CCR_ValSetter));
			}	
		}
		stp->CC_FLAG = !stp->CC_FLAG;		
}

/*******************************************************************************
 * @brief       定时器比较中断回调
 * @param       定时器通道
 * @retval      无
*******************************************************************************/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
			StepperInOC(&Stepper1);
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
			StepperInOC(&Stepper1);
	}		
}







