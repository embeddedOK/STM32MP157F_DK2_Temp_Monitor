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

volatile uint8_t wire_timeout = 0;
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

void wire1_delayUS(uint32_t delay)
{
	wire_timeout = 0;
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

typedef struct wire1_device
{
	uint64_t rom_code;
	uint64_t address;
	uint8_t power_mode;
	uint16_t temperature;
	uint8_t th_reg;
	uint8_t tl_reg;
	uint8_t config_reg;
}wire1_device;
//#define wire1_setZero()	(WIRE1_THERM_GPIO_Port->BSRR |= (WIRE1_THERM_Pin<<16))
#define wire1_setZero()	(WIRE1_THERM_GPIO_Port->MODER |= (1<<28))
//0x4000

//#define wire1_setOne()  (WIRE1_THERM_GPIO_Port->BSRR |= (WIRE1_THERM_Pin))
#define wire1_setOne()  (WIRE1_THERM_GPIO_Port->MODER &= ~(3<<28))

#define wire1_read() ((WIRE1_THERM_GPIO_Port->IDR>>14)&0x1)

#if 1
void wire1_debugToggle(void)
{

	for(int x=0; x<100; x++)
	{
		wire1_delayUS(1);
		wire1_setZero();
		wire1_delayUS(2);
		wire1_setOne();
	}
}
#endif

uint8_t wire1_resetPulse(void)
{
	uint32_t master_rx_time = 0; //Track time in Master RX mode, 480us Min
	uint8_t reading = 0;

	//Send Wire1 reset pulse width for 480us min
	wire1_setZero();
	wire1_delayUS(DS18B20_RESET_PULSE_WIDTH_MIN_US);
	wire1_setOne();


	//Check for DS18B20 TX Presence response
	wire1_delayUS(DS18B20_PRESENCE_PULSE_WAITS_MAX_US);
	master_rx_time += DS18B20_PRESENCE_PULSE_WAITS_MAX_US;

	reading = wire1_read();

	//Delay 15us and Read again
	//Check for DS18B20 TX Presence
	wire1_delayUS(DS18B20_PRESENCE_PULSE_RESAMPLE_US);
	master_rx_time += DS18B20_PRESENCE_PULSE_RESAMPLE_US;
	reading <<= 1;
	reading |= wire1_read();

	//Wait for remaining Master RX mode Time
	wire1_delayUS(DS18B20_RESET_PULSE_WIDTH_MIN_US-master_rx_time);

	return reading;
}

void wire1_sendBusStart(void)
{
//Wait for 1us recovery time in case we just sent a bit
	wire1_setOne();
	while(wire1_read() == 0);	//Wait for RC and bus to go high
	wire1_delayUS(DS18B20_SLOT_RECOVERY_US);
	wire1_setZero();
	wire1_delayUS(DS18B20_START_OF_SLOT_MIN_US);
	wire1_setOne();

}
void wire1_sendBusBit(uint8_t bit)
{
	wire1_sendBusStart();
	if(bit)
	{
		wire1_setOne();
	}
	else
	{
		wire1_setZero();
	}
	//Wait for DS18B20 Sample Time DS18B20_TIME_SLOT_MIN_US
	wire1_delayUS(DS18B20_TIME_SLOT_MIN_US);
	wire1_setOne();
}

uint8_t wire1_readBus(void)
{
	uint8_t reading = 0;
	wire1_sendBusStart();
	wire1_delayUS(SD18B20_MASTER_READ_SLOT_US-DS18B20_START_OF_SLOT_MIN_US);
	reading = wire1_read();
	wire1_delayUS(DS18B20_TIME_SLOT_MIN_US-SD18B20_MASTER_READ_SLOT_US);
	return reading;
}

void wire1_sendBusCMD(uint8_t cmd)
{
	for(int x=0; x<DS18B20_CMD_BIT_SIZE; x++)
	{
		wire1_sendBusBit(cmd&1);
		cmd >>=1;	//Right shift cmd
	}
}
////Calculate CRC based on CRC_POLY of 100110001, BITS are LSB First so
//__weak uint8_t wire1_calcCRC(uint8_t *message, uint8_t byteLen)	//Weak function as instead of slow bit wise loop we can override it to utilize CRC hw module
//{
//
//	uint16_t crc_poly = ((uint8_t)DS18B20_CRC_POLYNOMIAL)<<8;
//	uint16_t crc;
//	uint8_t xor_op=0;
//
//	crc = (*message)<<8;
//
//	for(int x=0; x<byteLen; x++)
//	{
//		crc &=0xFF00;
//		if(x+1 < byteLen)	//Add next message byte to end
//		{
//			crc |= *(message+x+1);
//		}
//
//		for(int y = 0; y<8; y++)
//		{
//			if(crc & 0x8000)	//MSb of value is 1, we can apply CRC XOR operation
//			{
//				xor_op = 1;
//			}
//			crc <<=1;	//Shift over value
//
//			if(xor_op) //Since CRC poly MSb is always 1 and MSb of value is 1, XOR will always be 0! So we can shift left and XOR 8bits or poly with 8bits of value!
//			{
//				xor_op = 0;
//				crc ^= crc_poly;
//			}
//		}
//	}
//	return (crc>>8);	//value should be CRC!
//}
////Calculate CRC based on CRC_POLY of 100110001, BITS are LSB First so
__weak uint8_t wire1_calcCRC(uint8_t *message, uint8_t byteLen)	//Weak function as instead of slow bit wise loop we can override it to utilize CRC hw module
{

	uint16_t crc_poly = ((uint8_t)DS18B20_CRC_POLYNOMIAL)<<8;
	uint16_t crc;

	uint8_t xor_op=0;

	crc = *(message+byteLen-1)<<8;
	for(int x=byteLen-1; x>=0; x--)
	{
		crc &=0xFF00;
		if(x > 0)	//Add next message byte to end
		{
			crc |= *(message+x-1);
		}

		for(int y = 0; y<8; y++)
		{
			if(crc & 0x8000)	//MSb of value is 1, we can apply CRC XOR operation
			{
				xor_op = 1;
			}
			crc <<=1;	//Shift over value

			if(xor_op) //Since CRC poly MSb is always 1 and MSb of value is 1, XOR will always be 0! So we can shift left and XOR 8bits or poly with 8bits of value!
			{
				xor_op = 0;
				crc ^= crc_poly;
			}
		}
	}
	return (crc>>8);	//value should be CRC!
}

uint8_t wire1_init(wire1_device *devices)
{
	uint64_t rom_code = 0;
	uint64_t address = 0;
	uint64_t address_contention = 0;
	uint8_t  crc_value = 0;


	uint8_t dev_cnt = 0;
	uint8_t reading = 0;


	uint8_t crc_cal = 0;

	reading = wire1_resetPulse();
	if(reading != 0)
	{
	 //No Device Detected
		return 0;
	}
	do
	{
		wire1_sendBusCMD(DS18B20_CMD_SEARCH_ROM);
		for(int x=0; x<DS18B20_ROM_CODE_BIT_SIZE; x++)
		{
			reading = wire1_readBus();	//Read TX Bit 0
			reading <<= 1;
			reading |= wire1_readBus();	//Read TX ~Bit 0
			switch(reading)
			{
				case 0:	//"00" Bus contention, Two Devices different address bits
					address_contention |= (uint64_t)1<<x;	//Store for later use, navigate 0 address first
					break;
				case 1:	//"01" Address 0 match
					//Nothing to do as Address bits default to 0
					break;
				case 2:	//"10" Address 1 match
					rom_code |= (uint64_t)1<<(DS18B20_ROM_CODE_BIT_SIZE-1-x);	//Store 1 in address, LSb first for CRC
					address |= (uint64_t)1<<x;	//Store 1 in address, LSb as LSb
					break;
				case 3:	//"11" No Devices responded, maybe error in master TX bit?
					return dev_cnt;
				default:
					break;
			}
			wire1_sendBusBit(((uint64_t)address>>x) & 1);
		}
		//got entire address!!
		//Check CRC
		crc_value = (uint8_t)rom_code; //Store message CRC
		rom_code >>= 8;	//Shift out recv'd crc code TODO: Parameterize
		crc_cal = wire1_calcCRC((uint8_t *)&rom_code, DS18B20_ADDRESS_BIT_SIZE/8);
		if(crc_cal == crc_value)
		{
			devices->rom_code = rom_code<<8 | crc_value;
			devices->address = address;
			devices++;
			dev_cnt++;
		}
	} while(dev_cnt < DS18B20_MAX_DEVICES);
	return 1;
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
//	htim5.Instance->CR1 |= TIM_CR1_CEN;	//Free running clock to bypass 3us start delay -_-
//  htim5.Instance->SR &= ~TIM_SR_UIF; //Reset UIF Flag before use to fix 2us irq trigger error?
//  htim5.Instance->CR1 |= TIM_CR1_URS;
//  htim5.Instance->DIER |= TIM_DIER_UIE;	//Enable TIM5 IRQ
  //	//Enable TIM5 UE IRQ
  //	htim5.Instance->DIER |= TIM_DIER_UIE;	//Enable TIM5 IRQ
  ////	//Enable Timer, should be free running, so only first call has 3uS TIM star delay
  //Free running TIM5 to bypass 3us start TIM delay -_-
  htim5.Instance->ARR = 0xFFFFFFFF;
  htim5.Instance->CR1 |= TIM_CR1_CEN;
  //Set WIRE1 GPIO as Input
  wire1_setOne();
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

  wire1_device dev[DS18B20_MAX_DEVICES] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint8_t ret = 0;

	OPENAMP_check_for_message();
	if (VirtUart0RxMsg)
	  {
		VirtUart0RxMsg = RESET;
		switch (VirtUart0ChannelBuffRx[0])
		{
		case '0' :
			ret = wire1_resetPulse();
			uint8_t message0[] = "Send WIRE1 Reset:  ";
			message0[sizeof(message0)-1] = ret+48;
			VIRT_UART_Transmit(&huart0, message0, sizeof(message0));
			break;

		case '1' :
			wire1_sendBusBit(1);
			uint8_t message1[] = "Send WIRE1 One";
			VIRT_UART_Transmit(&huart0, message1, sizeof(message1));
			break;

		case '2' :
			wire1_sendBusBit(0);
			uint8_t message2[] = "Send WIRE1 Zero";
			VIRT_UART_Transmit(&huart0, message2, sizeof(message2));
			break;

		case '3' :
			wire1_sendBusBit(0);
			wire1_sendBusBit(1);
			wire1_sendBusBit(0);
			uint8_t message3[] = "Send WIRE1 Zero/One/Zero";
			VIRT_UART_Transmit(&huart0, message3, sizeof(message3));
			break;

		case '4' :
			wire1_debugToggle();
			uint8_t message4[] = "Send debug toggle  ";
			VIRT_UART_Transmit(&huart0, message4, sizeof(message4));
			break;

		case '5':
			uint8_t message_address[16];
			uint8_t *msg_ptr = message_address;

			ret = wire1_init(dev);
			uint8_t message5[] = "Send wire1_init(): Num Dev = ";

			intToHexStr(ret, msg_ptr);
			VIRT_UART_Transmit(&huart0, message5, sizeof(message5));
			VIRT_UART_Transmit(&huart0, message_address, 2);

			if((ret > 0) && (ret != -1))
			{
				for(int x=56; x>=0; x-=8)
				{
					intToHexStr(dev[0].address>>x, msg_ptr);
					msg_ptr +=2;
				}
				VIRT_UART_Transmit(&huart0, "\nAddress=0x", sizeof("\nAddress=0x"));
				VIRT_UART_Transmit(&huart0, message_address, sizeof(message_address));
				VIRT_UART_Transmit(&huart0, "\n", sizeof("\n"));

				msg_ptr = message_address;
				for(int x=56; x>=0; x-=8)
				{
					intToHexStr(dev[0].rom_code>>x, msg_ptr);
					msg_ptr +=2;
				}
				VIRT_UART_Transmit(&huart0, "\nRomCode=0x", sizeof("\nRomCode=0x"));
				VIRT_UART_Transmit(&huart0, message_address, sizeof(message_address));
				VIRT_UART_Transmit(&huart0, "\n", sizeof("\n"));
			}
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//One shot timer mode!
//	if(HAL_TIM_Base_Stop_IT(htim) != HAL_OK)
//	{
//		log_err("htim Stop_IT failed.\r\n");
//		Error_Handler();
//	}
	if(htim == &htim5)
	{
		htim5.Instance->DIER &= ~TIM_DIER_UIE;	//Disable TIM5 IRQ
		wire_timeout = 1;
	}
}

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
