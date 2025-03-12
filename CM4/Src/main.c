/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  *	Based on OPENAMP_TTY DEMO FOR STM32MP157C-DK2
  *	https://github.com/STMicroelectronics/STM32CubeMP1/tree/525d2499658d817a9e669eb17e66390906954895/Projects/STM32MP157C-DK2/Applications/OpenAMP/OpenAMP_TTY_echo
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "virt_uart.h"
#include "openamp_log.h"
#include "DS18B20_REG.h"
#include "DS18B20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
#define EOK 0	//Everything was OK!

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc2;

IPCC_HandleTypeDef hipcc;

TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

//volatile uint8_t wire_timeout = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IPCC_Init(void);
static void MX_TIM5_Init(void);
static void MX_CRC2_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void intToHexStr(uint8_t val, uint8_t *buff)
{
	for(int x=4; x>=0; x-=4)
	{
		uint8_t tmp = ((val>>x)&0xF);
		if( tmp <= 9)
		{
			*buff = tmp+48;
		}
		else
		{
			*buff = tmp -10 +'A';
		}
		buff++;
	}
}

void DS18B20_delayUS(uint32_t delay)
{
	delay *= 10; //Timer pre divider is set to 20 making a tick 0.1uS
	if(delay > 1)
	{
		delay--;	//Subtract a cycle for zero cnt
	}

	//	//Reset the TIMx_CNT register
//	htim5.Instance->EGR |= TIM_EGR_UG;
//	Reset CNT
	htim5.Instance->CNT = 0;	//UIFCPY is read only so okay to clear

	while(htim5.Instance->CNT < delay);
}

//#define wire1_setZero()	(WIRE1_THERM_GPIO_Port->BSRR |= (WIRE1_THERM_Pin<<16))
void DS18B20_wire1_setZero(void)
{
	(WIRE1_THERM_GPIO_Port->MODER |= (1<<28));
}
//0x4000

//#define wire1_setOne()  (WIRE1_THERM_GPIO_Port->BSRR |= (WIRE1_THERM_Pin))
void DS18B20_wire1_setOne(void)
{
	(WIRE1_THERM_GPIO_Port->MODER &= ~(3<<28));
}

uint8_t DS18B20_wire1_read(void)
{
	return ((WIRE1_THERM_GPIO_Port->IDR>>14)&0x1);
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

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();
  }
  else
  {
    /* IPCC initialisation */
    MX_IPCC_Init();
    /* OpenAmp initialisation ---------------------------------*/
    MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_CRC2_Init();
  /* USER CODE BEGIN 2 */
//Enable Timer
  //Free running TIM5 to bypass 3us start TIM delay -_-
  htim5.Instance->ARR = 0xFFFFFFFF;
  htim5.Instance->CR1 |= TIM_CR1_CEN;
  //Set WIRE1 GPIO as Input
  DS18B20_wire1_setOne();
  //Set WIRE1 GPIO Low when output
  WIRE1_THERM_GPIO_Port->BSRR |= (WIRE1_THERM_Pin<<16);

  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
      log_err("VIRT_UART_Init UART0 failed.\r\n");
      Error_Handler();
  }
  /*Need to register callback for message reception by channels*/
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
	  log_err("VIRT_UART_Register_Callback() failed.\r\n");
  	  Error_Handler();
  }

  DS18B20 dev[DS18B20_MAX_DEVICES] = {0};
  DS18B20_BSP_FPTRS dev_fptrs = {
		  &DS18B20_delayUS,
		  &DS18B20_wire1_setZero,
		  &DS18B20_wire1_setOne,
		  &DS18B20_wire1_read
  };
  //Assign device specific function pointers

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	int8_t ret = 0;

	OPENAMP_check_for_message();
	if (VirtUart0RxMsg)
	  {
		VirtUart0RxMsg = RESET;
		switch (VirtUart0ChannelBuffRx[0])
		{
		case '0' :
			ret = DS18B20_resetPulse(dev);
			uint8_t message0[] = "Send WIRE1 Reset:  ";
			message0[sizeof(message0)-1] = ret+48;
			VIRT_UART_Transmit(&huart0, message0, sizeof(message0));
			break;

		case '1' :
			DS18B20_sendBusBit(dev, 1);
			uint8_t message1[] = "Send WIRE1 One";
			VIRT_UART_Transmit(&huart0, message1, sizeof(message1));
			break;

		case '2' :
			DS18B20_sendBusBit(dev, 0);
			uint8_t message2[] = "Send WIRE1 Zero";
			VIRT_UART_Transmit(&huart0, message2, sizeof(message2));
			break;

		case '3' :
			DS18B20_sendBusBit(dev, 0);
			DS18B20_sendBusBit(dev, 1);
			DS18B20_sendBusBit(dev, 0);
			uint8_t message3[] = "Send WIRE1 Zero/One/Zero";
			VIRT_UART_Transmit(&huart0, message3, sizeof(message3));
			break;

		case '4' :
			DS18B20_wire1_debugToggle(dev);
			uint8_t message4[] = "Send debug toggle  ";
			VIRT_UART_Transmit(&huart0, message4, sizeof(message4));
			break;

		case '5':
			uint8_t message_address[16];
			uint8_t *msg_ptr = message_address;

			ret = DS18B20_init(dev, &dev_fptrs, DS18B20_MAX_DEVICES);
			uint8_t message5[] = "Send wire1_init(): Num Dev = ";

			intToHexStr(ret, msg_ptr);
			VIRT_UART_Transmit(&huart0, message5, sizeof(message5));
			VIRT_UART_Transmit(&huart0, message_address, 2);

			if(ret > 0)
			{
				ret = DS18B20_convertTemperature(dev, 0);
				for(int x=56; x>=0; x-=8)
				{
					intToHexStr(dev[0].address>>x, msg_ptr);
					msg_ptr +=2;
				}
				VIRT_UART_Transmit(&huart0, "\nAddress=0x", sizeof("\nAddress=0x"));
				VIRT_UART_Transmit(&huart0, message_address, sizeof(message_address));

				if(ret != EOK)
				{
					VIRT_UART_Transmit(&huart0, "\nwire1_convertT() FAILED", sizeof("\nwire1_convertT() FAILED"));
				}
				else
				{
					int16_t temperature;
					ret = DS18B20_getTemperature(dev, &temperature);

					if(ret != EOK)
					{
						VIRT_UART_Transmit(&huart0, "\nwire1_getT() FAILED", sizeof("\nwire1_convertT() FAILED"));
					}
				}
			}
			else if(ret < 0)
			{

			}
			break;
		default:
			VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
			break;
		}
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC2_Init(void)
{

  /* USER CODE BEGIN CRC2_Init 0 */

  /* USER CODE END CRC2_Init 0 */

  /* USER CODE BEGIN CRC2_Init 1 */

  /* USER CODE END CRC2_Init 1 */
  hcrc2.Instance = CRC2;
  hcrc2.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc2.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc2.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc2.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc2.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC2_Init 2 */

  /* USER CODE END CRC2_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 20;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIRE1_THERM_GPIO_Port, WIRE1_THERM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : WIRE1_THERM_Pin */
  GPIO_InitStruct.Pin = WIRE1_THERM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WIRE1_THERM_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Removed IRQ and use free running clock as IRQ and timer restart incurs 3uS delay -_-
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//
////	if(htim == &htim5)
////	{
////		htim5.Instance->DIER &= ~TIM_DIER_UIE;	//Disable TIM5 IRQ
////	}
//}

void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{

    log_info("Msg received on VIRTUAL UART0 channel:  %s \n\r", (char *) huart->pRxBuffPtr);

    /* copy received msg in a variable to sent it back to master processor in main infinite loop*/
    VirtUart0ChannelRxSize = huart->RxXferSize < MAX_BUFFER_SIZE? huart->RxXferSize : MAX_BUFFER_SIZE-1;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    VirtUart0RxMsg = SET;
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
