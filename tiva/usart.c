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

//ʵ��ʹ��ʱ���Ը�д�ɺ���
int main(void)
{
  SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_OSC|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//����ϵͳʱ��Ϊ16MHz
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //ʹ��B�˿�
  //����GPIO����
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinConfigure(GPIO_PB1_U1TX);//ʹ�ܸ���
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);//�����ź�
	//���ô��ڲ����ʡ�У��λ��ֹͣλ�Ͳ���
  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	//ʹ��FIFO�����ݻ�������
	UARTFIFOEnable(UART1_BASE);
	//���շ��͵�FIFO��Ϊ1/4*16�ֽ�
	UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX2_8,UART_FIFO_RX2_8);
	//ʹ�ܴ��ڵĽ��պͽ��ճ�ʱ�ж�
	UARTIntEnable(UART1_BASE,UART_INT_RX|UART_INT_RT);
	//ע���жϺ���
	UARTIntRegister(UART1_BASE,USART1_IRAHandler);
	//�����ж����ȼ�
	IntPrioritySet(INT_UART1,0);
	//�����ж�
	IntEnable(INT_UART1);
	IntMasterEnable();
	
	//ʹ�ܴ���
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
	//�ж�UART�Ƿ����ַ�δ��ȡ
	while(UARTCharsAvail(UART1_BASE))
	{
		//������ַ�Ϊ��ȡ��ȡ����ʹ��UARTCharGetNonBlocking��ֹ�ȴ�
		re_buf=UARTCharGetNonBlocking(UART1_BASE);
		UARTCharPutNonBlocking(UART1_BASE,re_buf);
	}
	
}
