/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "datascope_dp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//////////////////////////ȫ�ֱ����Ķ���////////////////////////////////////  
float pitch,roll,yaw; 								  			 //ŷ����(��̬��)
short aacx,aacy,aacz;													 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;											 //������ԭʼ����
float temp = 0; 								  								 //�¶�
int   Encoder_Left,Encoder_Right;         		 //���ұ��������������
int 	Moto1=0,Moto2=0;												 //������������ո��������PWM
int   Balance_Pwm,Velocity_Pwm,Turn_Pwm;
float Mechanical_angle = -4.2; 
uint8_t flag = 0;
/****************************��λ������*********************************/   
unsigned char i;          										 //��������
unsigned char Send_Count; 										 //������Ҫ���͵��ֽ���
/**********************************************************************/
uint8_t led_set;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  delay_init(72);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
  MPU_Init();					    			 //=====��ʼ��MPU6050
	mpu_dmp_init();								 //=====��ʼ��MPU6050��DMPģʽ
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    if(flag == 1)
    {
      mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===�õ�ŷ���ǣ���̬�ǣ�������
      MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===�õ�����������
      MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
    	Encoder_Left = -Read_Encoder(2);                           //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
      Encoder_Right = Read_Encoder(4);                           //===��ȡ��������ֵ
      Balance_Pwm = balance_UP(pitch,Mechanical_angle,gyroy);   //===ƽ�⻷PID����	
      Velocity_Pwm = velocity(Encoder_Left,Encoder_Right);       //===�ٶȻ�PID����
      if(1==Flag_Left||1==Flag_Right)   
      {        
        Turn_Pwm = turn(Encoder_Left,Encoder_Right,gyroz);        //===ת��PID����
      }
      else
      {
        Turn_Pwm=0.5*gyroz;
      }
      Moto1 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;                 //===�������ֵ������PWM
      Moto2 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;                 //===�������ֵ������PWM
      Xianfu_Pwm();  																					 //===PWM�޷�
      Turn_Off(pitch,aacz);																 //===���Ƕ��Լ���ѹ�Ƿ�����
      Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ��� 
      flag = 0;
      
    }
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
  if(GPIO_Pin == GPIO_PIN_5)
  { 
    led_set++;
    if(led_set == 8)
    {
      led_set = 0;
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    } 
    flag = 1;
//    mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===�õ�ŷ���ǣ���̬�ǣ�������
//		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===�õ�����������

//		Balance_Pwm = balance_UP(pitch,Mechanical_angle,gyroy);   //===ƽ�⻷PID����	
//		Moto1 = Balance_Pwm;                                      //===�������ֵ������PWM
//		Moto2 = Balance_Pwm;                                      //===�������ֵ������PWM
//		Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���     
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
