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

void USART1_IRAHandler(void);

//实际使用时可以改写成函数
int main(void)
{
  SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_OSC|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//配置系统时钟为16MHz
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //使能B端口
  //配置GPIO引脚
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinConfigure(GPIO_PB1_U1TX);//使能复用
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);//分配信号
	//配置串口波特率、校验位、停止位和步长
  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	//使能FIFO（数据缓存区）
	UARTFIFOEnable(UART1_BASE);
	//接收发送的FIFO都为1/4*16字节
	UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX2_8,UART_FIFO_RX2_8);
	//使能串口的接收和接收超时中断
	UARTIntEnable(UART1_BASE,UART_INT_RX|UART_INT_RT);
	//注册中断函数
	UARTIntRegister(UART1_BASE,USART1_IRAHandler);
	//设置中断优先级
	IntPrioritySet(INT_UART1,0);
	//开启中断
	IntEnable(INT_UART1);
	IntMasterEnable();
	
	//使能串口
	UARTEnable(UART1_BASE);
	
  UARTCharPut(UART1_BASE, 'g');
  while(1){}
  return 0;
}

void USART1_IRAHandler(void)
{
	unsigned char re_buf;
	uint32_t status=UARTIntStatus(UART1_BASE,true);
	UARTIntClear(UART1_BASE,status);
	//判断UART是否有字符未读取
	while(UARTCharsAvail(UART1_BASE))
	{
		//如果有字符为读取就取出，使用UARTCharGetNonBlocking防止等待
		re_buf=UARTCharGetNonBlocking(UART1_BASE);
		UARTCharPutNonBlocking(UART1_BASE,re_buf);
	}
	
}
