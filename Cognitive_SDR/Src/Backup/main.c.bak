/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "S2LP_radio.h"
#include "MCU_interface.h"

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define BUFSIZE (32)
extern SPI_HandleTypeDef SpiHandle;
volatile uint8_t start_Tx = RESET;  /* flag to trigger radio transmission after push button */
volatile uint8_t finished_Rx = RESET;  /* flag to indicate data has been received */
volatile uint8_t cRxData = 0;  /* received data buffer */
uint16_t idx;
S2LPIrqs xIrqStatus;
uint8_t vectcRxBuff[BUFSIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);

	return len;
}

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  SpiHandle = hspi1;

     /* reset procedure of S2-LP chip */

     /* Puts high the GPIO connected to shutdown pin */
     HAL_GPIO_WritePin(GPIOA, M2S_GPIO_SDN_Pin, GPIO_PIN_SET);
     /* Puts low the GPIO connected to shutdown pin */
     HAL_GPIO_WritePin(GPIOA, M2S_GPIO_SDN_Pin, GPIO_PIN_RESET);
     /* Delay to allow the circuit POR, about 700 us */
     HAL_Delay(1) ;

     /* set frequency of Xtal (50MHz) */
     S2LPRadioSetXtalFrequency(50000000);

     /* initialize Radio Settings */
       /* S2LP Radio config */

     SRadioInit xRadioInit = {
     868000000, /* frequency base */
     0x00, /* 2-FSK modulation */
     40000, /* data rate */
     20000, /* frequency deviation */
     100000 /* channel filter */
   };

   PktBasicInit xBasicInit={
     16, /* PREAMBLE_LENGTH, */
     32, /* SYNC_LENGTH, */
     0x88888888, /* SYNC_WORD, */
     S_ENABLE, /* VARIABLE_LENGTH, */
     S_DISABLE, /* EXTENDED_LENGTH_FIELD, */
     0x20, /* CRC_MODE, */
     S_DISABLE, /* EN_ADDRESS, */
     S_DISABLE, /* EN_FEC, */
     S_ENABLE /* EN_WHITENING */
   };

   SGpioInit xGpioIRQ={
     0x03, /* @ of GPIO3 register */
     0x02, /* setup GPIO as digital output */
     0x00  /* output function as IRQ */
   };

   /**
   * @brief Tx buffer declaration: data to transmit
   */
     uint8_t vectcTxBuff[BUFSIZE]= "Trainer forward";  /* Put your ID i.e. name, up to BUFSIZE long */

     /* Rx & Tx common radio initialization */
     S2LPRadioInit(&xRadioInit);

       /* S2LP Packet config */
     S2LPPktBasicInit(&xBasicInit);

     uint8_t tmpBuffer[4];
     S2LPSpiReadRegisters(SYNT3_ADDR, 4, tmpBuffer);

       /* payload length config */
     S2LPPktBasicSetPayloadLength(BUFSIZE);

     /* S2LP IRQ config */
     S2LPGpioInit(&xGpioIRQ);

     /* S2LP IRQs enable */
     S2LPGpioIrqDeInit(NULL);
     S2LPGpioIrqConfig(TX_DATA_SENT , S_ENABLE);
	//S2LPGpioIrqConfig(RX_DATA_DISC,S_ENABLE);
     S2LPGpioIrqConfig(RX_DATA_READY,S_ENABLE);

     /* flush the TX & RX FIFO */
     S2LPCmdStrobeFlushRxFifo();
     S2LPCmdStrobeFlushTxFifo();

     /* start Rx by default */
     S2LPCmdStrobeRx();

     printf("\n\rApplication started\n\r");
     printf("\n\rPress user button to TX...\n\r");

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (start_Tx!=RESET)
	{
	  /* prepare transmission */
	  /* abort continuous Reception first */
	  S2LPCmdStrobeSabort();

	  /* add short delay to ensure stable S2-LP stable before transmission */
	  HAL_Delay(1);

	  /* fit the TX FIFO */
	  S2LPCmdStrobeFlushTxFifo();
	  S2LPSpiWriteFifo(BUFSIZE, vectcTxBuff);

	  /* send the TX command */
	  S2LPCmdStrobeTx();
	  printf("Packet transmitted\n\r");
	  HAL_Delay(100);
	  /* clear application TX flag */
	  start_Tx = RESET;
	 }

/* RX debug printf section */
#if 0
	 if (finished_Rx!=RESET)
	 {
	   /* received data print out section */
	   /* character format print out */
	   printf("RX CHAR: ");
	   for(idx=0;idx<cRxData;idx++)
	   {
	     if (vectcRxBuff[idx]>=0x20)  /* print out characters only */
	     {
	       printf("%c",vectcRxBuff[idx]);
	     }
	   }
	   printf("\n\r");
	   /* hex format print out */
	   printf("RX HEX : ");
	   for(idx=0;idx<cRxData;idx++)
	   {
	      printf("%02X ",vectcRxBuff[idx]);
	   }
	   printf("\n\r");
	   /* clear application RX flag */
	   finished_Rx = RESET;
	 }
#endif

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_S2_LP_Pin|NUCLEO_LED1_PIN_Pin|M2S_GPIO_SDN_Pin|SPI_CS_EEPROM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON1_PIN_Pin M2S_GPIO_3_Pin */
  GPIO_InitStruct.Pin = BUTTON1_PIN_Pin|M2S_GPIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_S2_LP_Pin NUCLEO_LED1_PIN_Pin M2S_GPIO_SDN_Pin SPI_CS_EEPROM_Pin */
  GPIO_InitStruct.Pin = SPI_CS_S2_LP_Pin|NUCLEO_LED1_PIN_Pin|M2S_GPIO_SDN_Pin|SPI_CS_EEPROM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : M2S_GPIO_1_Pin */
  GPIO_InitStruct.Pin = M2S_GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M2S_GPIO_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
