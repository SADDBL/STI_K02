#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
/**
 * main.c
 */

void Key2IntHandler()
{
    uint32_t i=GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, i);
    //GPIOPinRead()返回值
    //当端口为高电平时返回值的低8位：00010000，即16。当端口为低电平时返回0。
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_2);  //清除中断标志位
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_OSC|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//配置系统时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //使能B端口
    //配置GPIO端口
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_1);  //PB1设为输出模式
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);  //PB2设为输入模式
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);    //PB2设为弱上拉模式，驱动电流为2mA
	//设置B端口中断
    GPIOIntRegister(GPIO_PORTB_BASE, Key2IntHandler);   //注册中断处理句柄
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_2 , GPIO_FALLING_EDGE); //设置下降沿触发
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);//中断使能
//    IntEnable(INT_GPIOB_TM4C123);
    while(1){};
}
