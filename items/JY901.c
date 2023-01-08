#define BufLen 33 
//JY901每次发送出来的数据都是11位的，其中第一位位帧头，第二位为判断属于的类型
//第一位始终为0x55，第二位分别为0x51、0x52、0x53
//其中加速度的类位为0x51，角速度为0x52，欧拉角为0x53

//陀螺仪数据结构体
//接收数据使用DMA空闲中断
typedef struct _JY901_DMA
{
	uint8_t DataBuf[BufLen];
	uint8_t Flag;
}_JY901;

//陀螺仪计算角度
//此处计算的是角度X，角度Y和Z数据顺序后延
//使用前需要确定极性
float JY901_Ang_Cal(_JY901 GY_Ins)
{
	float X_Angle = ((short)(GY_Ins.DataBuf[25]<<8|GY_Ins.DataBuf[24]))*180/32768.0;
	return X_Angle;
}

/*** 三种数据数据结算参考***/
/*
case 0x51:	
	memcpy(&stcAcc,&JY901_data.RxBuffer[2 + i*11],8);
	for(uint8_t j = 0; j < 3; j++) JY901_data.acc.a[j] = (float)stcAcc.a[j]/32768*16;					//官方加速度解算
	break;
case 0x52:	
	memcpy(&stcGyro,&JY901_data.RxBuffer[2 + i*11],8);
	for(uint8_t j = 0; j < 3; j++) JY901_data.w.w[j] = (float)stcGyro.w[j]/32768*2000;					//官方角速度解算
	break;
case 0x53:	
	memcpy(&stcAngle,&JY901_data.RxBuffer[2 + i*11],8);
	for(uint8_t j = 0; j < 3; j++) JY901_data.angle.angle[j] = (float)stcAngle.Angle[j]/32768*180;		//官方角度解算
	break;
*/

//DMA接收陀螺仪数据
void USART2_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))//idle标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
		HAL_UART_DMAStop(&huart2); //  停止DMA传输，防止
		//下面开始对收到的数据进行解算
		GY25_Ang_Cal(GY_Ins);
	}
	HAL_UART_Receive_DMA(&huart2,GY_Ins.DataBuf,BufLen);//重新打开DMA接收
  HAL_UART_IRQHandler(&huart2);
}