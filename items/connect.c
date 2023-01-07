# include <stdio.h>
# include <usart.h>
# include <connect.h>

/*�ض���printf������**/
//ʹ��ǰ�ض��򵽿����Ĵ���
//����HAL_UART_Transmit()�����ĵ�һ������
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}

//���ն����ַ���
//�����У��������ݸ�ʽΪfxxx
uint8_t  RxLine=0; //��¼�������ݳ���
uint8_t flag=0;
uint8_t rxbufdata[4]={0};//�������ݳ���Ϊ4
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
			//�жϿ�ͷ��ĸ�Ƿ�Ϊf
				flag=1;
				RxLine=0;
			}
		}
		HAL_UART_Receive_IT(&huart3, (uint8_t *)RecieveBuffer, 1);
	}
}