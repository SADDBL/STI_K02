#include "connect.h"

uint8_t RecieveBuffer[1];
uint8_t usart2_buf[6] = {0};
uint8_t RxLen2;
uint8_t flag2 = 0;

int stepper_usart_angle[2]={0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	
	/***** USART2-调试串口 *****/
	if(huart->Instance == USART2)
	{
		if(flag2!=0){
			usart2_buf[RxLen2++] = *RecieveBuffer;
			if(RxLen2==6||*RecieveBuffer=='z'){
				int angle=0,index=100,i=0;
				RxLen2=0;
				flag2=0;
				for(i=0;usart2_buf[i]!='z';i++){
					if(i==0){
						switch(usart2_buf[0])
						{
							case 'a':stepper_usart_angle[0] = 1;
											 break;
							case 'b':stepper_usart_angle[0] = 2;
											 break;
							case 'c':stepper_usart_angle[0] = 3;
											 break;
							case 'd':stepper_usart_angle[0] = 4;
											 break;
							default:printf("输入格式有误\r\n");
											break;
						}
						continue;
					}
					if(i==1&&usart2_buf[1]=='-'){
						index*=-1;
						continue;
					}
					angle += index*(usart2_buf[i]-'0');
					index/=10;
				}
				stepper_usart_angle[1] = angle;
				printf("%d\r\n",stepper_usart_angle[1]);
				printf("%d\r\n",stepper_usart_angle[0]);
			}
		}
		else{
			if(*RecieveBuffer=='a'||*RecieveBuffer=='b'||*RecieveBuffer=='c'||*RecieveBuffer=='d'){
				flag2 = 1;
				RxLen2 = 0;
				usart2_buf[RxLen2++] = *RecieveBuffer;
			}
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t *)RecieveBuffer, 1);
	}
	
	
}

