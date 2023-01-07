# include <stdio.h>
# include <usart.h>
# include <connect.h>

/*重定向printf到串口**/
//使用前重定向到开启的串口
//更改HAL_UART_Transmit()函数的第一个参数
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}

//接收定长字符串
//此例中，接收数据格式为fxxx
uint8_t  RxLine=0; //记录接收数据长度
uint8_t flag=0;
uint8_t rxbufdata[4]={0};//接收数据长度为4
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	if(huart->Instance == USART3)
	{
		if(flag!=0){
			rxbufdata[RxLine++]=*RecieveBuffer;
			if(RxLine==3){
				RxLine=0;
				flag=0;
			}
		}else{
			if(*RecieveBuffer=='f'){
			//判断开头字母是否为f
				flag=1;
				RxLine=0;
			}
		}
		HAL_UART_Receive_IT(&huart3, (uint8_t *)RecieveBuffer, 1);
	}
}