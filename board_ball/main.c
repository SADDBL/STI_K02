/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Control.h"
#include "connect.h"
#include "oled.h"
#include "Hardware.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define x0 300
#define y0 300

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,10);
	return ch;
}

void delay_us(uint32_t us);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int OC_Channel1_Pulse,OC_Channel2_Pulse;//����Ƚ�Pulseֵ���������Ƶ�ʣ�f=1MHz/Pulse
int OC_Channel1_Duty,OC_Channel2_Duty;//����Ƚ�Dutyֵ������ռ�ձȣ���Duty%

//�������������ֵ
int keyborad_data;

//С��Ŀ��λ��(x,y)
int x_target,y_target;

//С��ǰλ��(x,y)
int x_cur,y_cur;

//PID�������Ĳ�����д��ȫ�ַ����޸�
float kp1=0.1,ki1=0,kd1=0;
float kp2=0.1,ki2=0,kd2=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,RecieveBuffer,1);
	HAL_UART_Receive_IT(&huart3,RecieveBuffer3,1);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE );
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	OLED_Init();
	OLED_DisPlay_On();
	oled_pid_para_dis(1);
	OC_Channel1_Duty=50;
	OC_Channel2_Duty=50;
	
	//PID�������ṹ���ʼ��
	pid_init(&pid_controler1,kp1,ki1,kd1,6,-6);
	pid_init(&pid_controler2,kp2,ki2,kd2,6,-6);
	
	//��������ṹ���ʼ��
	stepper_init(&motor1,GPIO_PIN_6,GPIOA,GPIO_PIN_15,TIM_CHANNEL_1,&pid_controler1,1);
	stepper_init(&motor2,GPIO_PIN_7,GPIOB,GPIO_PIN_3,TIM_CHANNEL_2,&pid_controler2,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7){
		int i,data=0;
		LDOut_High;
		delay_us(100);
		LDOut_Low;
		for(i=0;i<16;i++){
			delay_us(1);
			if(DAin==GPIO_PIN_SET) data = 16 - i;
			CKOut_High;
			delay_us(1);
			CKOut_Low;
		}
		keyborad_data=data;
//		printf("%d\r\n",keyborad_data);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	}
}

//��ʱ���жϺ���
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int stepper_No_last,stepper_angle_last;
	static int x_last = x0, y_last = y0;
  
	/***** TIM1-��ʱ���ж�-50Hz *****/
	if(htim->Instance == TIM1){
		/***** PID���� *****/
		pid_dangle(&motor2,270);
		pid_dangle(&motor1,270);
//		printf("out=%0.2f\t",motor1.pid_concroler->output);
//		printf("err=%0.2f\t",motor1.pid_concroler->err);
//		printf("cur=%0.2f\t",(motor1.step_record*MICRO_STEP_ANGLE-45));
//		printf("x=%d,y=%d\r\n",x_cur,y_cur);
//		if(x_cur==x_last&&y_cur==y_last);
//		else{
//			if(x_cur!=x_last){
//			//	pid_dangle(&motor2,270);
//				x_last=x_cur;
//			}
//			if(y_cur!=y_last){
//				pid_dangle(&motor1,270);
//				
////				printf("out=%0.2f\t",motor1.pid_concroler->output);
////				printf("err=%0.2f\t",motor1.pid_concroler->err);
////				printf("cur=%0.2f\t",(motor1.step_record*MICRO_STEP_ANGLE-45));
////				printf("x=%d,y=%d\r\n",x_cur,y_cur);
//				y_last=y_cur;
//			}
//		}
		
		/***** ���ڿ��� *****/
		if(stepper_angle_last==stepper_usart_angle[1]&&(stepper_No_last==stepper_usart_angle[0]));
		else{
			switch((int)stepper_usart_angle[0]){
				case 1:
					//stepper_to_angle(&motor1,stepper_usart_angle[1],270);
				board_y_angle(stepper_usart_angle[1],270);
				stepper_angle_last = stepper_usart_angle[1];
				stepper_No_last = stepper_usart_angle[0];
					break;
				case 2:
					//stepper_to_angle(&motor2,stepper_usart_angle[1],270);
				board_x_angle(stepper_usart_angle[1],270);
				stepper_angle_last = stepper_usart_angle[1];
				stepper_No_last = stepper_usart_angle[0];
					break;
				default: break;
			}
		}
	}
}

//����Ƚ��жϺ���
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t OC_Count = 0;
	
	/***** TIM4-����Ƚ� *****/
  if(htim->Instance == TIM4)
  {
		OC_Count = __HAL_TIM_GET_COUNTER(htim);
		//No1 ���
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
			
      if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,motor1.Stp_pin))
      {
				if(motor1.pulsenum_left==0)HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_1);
				else{
					if(motor1.stepper_dir==CW){
						motor1.pulsenum_left--;
						motor1.step_record = (motor1.step_record+1)%SPR;
					}
				}
			OC_Channel1_Pulse = (1000000*MICRO_STEP_ANGLE)/motor1.Anl_v;
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,OC_Count + OC_Channel1_Pulse - OC_Channel1_Duty*OC_Channel1_Pulse/100);
      }
      else	//������
      {
				//�Ѿ��ﵽ��Ҫ��������������ֹͣ��������
				if(motor1.pulsenum_left==0)HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_1);
				//δ�ﵽ���������٣�ͬʱ�ı���������¼
				else {
					if(motor1.stepper_dir==CCW){
						motor1.pulsenum_left--;
						if(motor1.step_record==0) motor1.step_record = SPR-1;
						else motor1.step_record--;
					}
				}
				//���ò��β���
				OC_Channel1_Pulse = (1000000*MICRO_STEP_ANGLE)/motor1.Anl_v;
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,OC_Count + OC_Channel1_Duty*OC_Channel1_Pulse/100);
      }
    }
		//No2 ���
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
			
      if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,motor2.Stp_pin))
      {
				if(motor2.pulsenum_left==0)HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_2);
				else{
					if(motor2.stepper_dir==CW){
						motor2.pulsenum_left--;
						motor2.step_record = (motor2.step_record+1)%SPR;
					}
				}
			OC_Channel2_Pulse = (1000000*MICRO_STEP_ANGLE)/motor2.Anl_v;
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,OC_Count + OC_Channel2_Pulse - OC_Channel2_Duty*OC_Channel2_Pulse/100);
      }
      else	//������
      {
				//�Ѿ��ﵽ��Ҫ��������������ֹͣ��������
				if(motor2.pulsenum_left==0)HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_2);
				//δ�ﵽ���������٣�ͬʱ�ı���������¼
				else {
					if(motor2.stepper_dir==CCW){
						motor2.pulsenum_left--;
						if(motor2.step_record==0) motor2.step_record = SPR-1;
						else motor2.step_record--;
					}
				}
				//���ò��β���
				OC_Channel2_Pulse = (1000000*MICRO_STEP_ANGLE)/motor2.Anl_v;
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,OC_Count + OC_Channel2_Duty*OC_Channel2_Pulse/100);
      }
    }
		
	}
}

//usart3�жϺ�����ʹ��DMA����

// ����ʱ����������F1ϵ��
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */