#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "timer.h"

void TIMER_IERHandler(void);
uint32_t sys;
//实际使用时可以改写成函数
int main(void)
{
	//使用PLL输出频率200MHz，第一个系数4分频，得到50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
 
	//使能定时器TIMER0，16/32bit
    SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER0);
	//配置定时器，分频作为定时器的周期
	TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC_UP);
	//配置装载值，目标频率100Hz
	TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/100-1);
	IntMasterEnable();
	//中断注册
	TimerIntRegister( TIMER0_BASE,  TIMER_A, TIMER_IERHandler);
	//使能time0的定时器A为超时中断
	TimerIntEnable( TIMER0_BASE,  TIMER_TIMA_TIMEOUT);
    //设置中断优先级
	IntPrioritySet( INT_TIMER0A,  0);
    //使能中断
	IntEnable( INT_TIMER0A);
  	//使能定时器
	TimerEnable( TIMER0_BASE,  TIMER_A);
	
  	while(1){}
  	return 0;
}

void TIMER_IERHandler(void)
{
	static int i=0;
	uint32_t status=TimerIntStatus( TIMER0_BASE,  true);
	//清除标志位
	TimerIntClear( TIMER0_BASE,  status);

}

