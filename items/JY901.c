#define BufLen 33 
//JY901ÿ�η��ͳ��������ݶ���11λ�ģ����е�һλλ֡ͷ���ڶ�λΪ�ж����ڵ�����
//��һλʼ��Ϊ0x55���ڶ�λ�ֱ�Ϊ0x51��0x52��0x53
//���м��ٶȵ���λΪ0x51�����ٶ�Ϊ0x52��ŷ����Ϊ0x53

//���������ݽṹ��
//��������ʹ��DMA�����ж�
typedef struct _JY901_DMA
{
	uint8_t DataBuf[BufLen];
	uint8_t Flag;
}_JY901;

//�����Ǽ���Ƕ�
//�˴�������ǽǶ�X���Ƕ�Y��Z����˳�����
//ʹ��ǰ��Ҫȷ������
float JY901_Ang_Cal(_JY901 GY_Ins)
{
	float X_Angle = ((short)(GY_Ins.DataBuf[25]<<8|GY_Ins.DataBuf[24]))*180/32768.0;
	return X_Angle;
}

/*** �����������ݽ���ο�***/
/*
case 0x51:	
	memcpy(&stcAcc,&JY901_data.RxBuffer[2 + i*11],8);
	for(uint8_t j = 0; j < 3; j++) JY901_data.acc.a[j] = (float)stcAcc.a[j]/32768*16;					//�ٷ����ٶȽ���
	break;
case 0x52:	
	memcpy(&stcGyro,&JY901_data.RxBuffer[2 + i*11],8);
	for(uint8_t j = 0; j < 3; j++) JY901_data.w.w[j] = (float)stcGyro.w[j]/32768*2000;					//�ٷ����ٶȽ���
	break;
case 0x53:	
	memcpy(&stcAngle,&JY901_data.RxBuffer[2 + i*11],8);
	for(uint8_t j = 0; j < 3; j++) JY901_data.angle.angle[j] = (float)stcAngle.Angle[j]/32768*180;		//�ٷ��ǶȽ���
	break;
*/

//DMA��������������
void USART2_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))//idle��־����λ
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//�����־λ
		HAL_UART_DMAStop(&huart2); //  ֹͣDMA���䣬��ֹ
		//���濪ʼ���յ������ݽ��н���
		GY25_Ang_Cal(GY_Ins);
	}
	HAL_UART_Receive_DMA(&huart2,GY_Ins.DataBuf,BufLen);//���´�DMA����
  HAL_UART_IRQHandler(&huart2);
}