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
    //GPIOPinRead()����ֵ
    //���˿�Ϊ�ߵ�ƽʱ����ֵ�ĵ�8λ��00010000����16�����˿�Ϊ�͵�ƽʱ����0��
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_2);  //����жϱ�־λ
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_OSC|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//����ϵͳʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //ʹ��B�˿�
    //����GPIO�˿�
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_1);  //PB1��Ϊ���ģʽ
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);  //PB2��Ϊ����ģʽ
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);    //PB2��Ϊ������ģʽ����������Ϊ2mA
	//����B�˿��ж�
    GPIOIntRegister(GPIO_PORTB_BASE, Key2IntHandler);   //ע���жϴ�����
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_2 , GPIO_FALLING_EDGE); //�����½��ش���
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);//�ж�ʹ��
//    IntEnable(INT_GPIOB_TM4C123);
    while(1){};
}
