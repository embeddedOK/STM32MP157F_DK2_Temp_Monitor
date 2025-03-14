#include "main.h"
#include "openamp.h"

#include "BSP_main.h"
#include "virt_uart.h"
#include "openamp_log.h"
#include "DS18B20_REG.h"
#include "DS18B20.h"

#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
#define EOK 0	//Everything was OK!

CRC_HandleTypeDef *BSP_hcrc2;
IPCC_HandleTypeDef *BSP_hipcc;
TIM_HandleTypeDef *BSP_htim5;

VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
	BSP_htim5->Instance->CNT = 0;	//UIFCPY is read only so okay to clear

	while(BSP_htim5->Instance->CNT < delay);
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

DS18B20_wire1_interface wire1_fptrs =
{
		&DS18B20_delayUS,
		&DS18B20_wire1_setZero,
		&DS18B20_wire1_setOne,
		&DS18B20_wire1_read,
		0
};
DS18B20 devices[DS18B20_MAX_DEVICES] = {0};

void BSP_main(BSP_ARGS *argv)
{
	BSP_hcrc2 = argv->hcrc2;
	BSP_hipcc = argv->hipcc;
	BSP_htim5 = argv->htim5;

	BSP_htim5->Instance->ARR = 0xFFFFFFFF;
	BSP_htim5->Instance->CR1 |= TIM_CR1_CEN;
	//Set WIRE1 GPIO as Input
	DS18B20_wire1_setOne();
	//Set WIRE1 GPIO Low when output
	WIRE1_THERM_GPIO_Port->BSRR |= (WIRE1_THERM_Pin<<16);

	if (VIRT_UART_Init(&huart0) != VIRT_UART_OK)
	{
		log_err("VIRT_UART_Init UART0 failed.\r\n");
		Error_Handler();
	}
	/*Need to register callback for message reception by channels*/
	if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
	{
		log_err("VIRT_UART_Register_Callback() failed.\r\n");
		Error_Handler();
	}



	uint8_t dev_cnt = 0;
	//Assign device specific function pointers
	while (1)
	{
		int8_t ret = 0;
		uint8_t message_address[16];
		uint8_t *msg_ptr = message_address;
		OPENAMP_check_for_message();
		if (VirtUart0RxMsg)
		{
			VirtUart0RxMsg = RESET;
			switch (VirtUart0ChannelBuffRx[0])
			{
			case '0' :
				ret = DS18B20_init(devices, &wire1_fptrs, DS18B20_MAX_DEVICES);
				VIRT_UART_Transmit(&huart0, "\nSend wire1_init(): ", sizeof("\nSend wire1_init(): "));
				intToHexStr(ret, msg_ptr);

				if(ret >= 0)
				{
					dev_cnt = ret;
					VIRT_UART_Transmit(&huart0, "Num Dev = 0x", sizeof("Num Dev = 0x"));
					VIRT_UART_Transmit(&huart0, message_address, 2);
					for(int x=0; x< dev_cnt; x++)
					{
						msg_ptr = message_address;
						intToHexStr(x, msg_ptr);
						VIRT_UART_Transmit(&huart0, "\nDevice 0x", sizeof("\nDevice 0x"));
						VIRT_UART_Transmit(&huart0, message_address, 2);
						for(int y=56; y>=0; y-=8)
						{
							intToHexStr(devices[x].address>>y, msg_ptr);
							msg_ptr +=2;
						}
						VIRT_UART_Transmit(&huart0, " Address=0x", sizeof("\nAddress=0x"));
						VIRT_UART_Transmit(&huart0, message_address, sizeof(message_address));
					}
				}
				else
				{
					VIRT_UART_Transmit(&huart0, "ERROR = 0x", sizeof("ERROR = 0x"));
					VIRT_UART_Transmit(&huart0, message_address, 2);
				}

				break;

			case '1':
				VIRT_UART_Transmit(&huart0, "\nStart Temperature Conversion per device", sizeof("\nStart Temperature Conversion per device"));

				for(int x=0; x< dev_cnt; x++)
				{
					ret = DS18B20_convertTemperatureDevice(&devices[x],0);
					VIRT_UART_Transmit(&huart0, "\nDev 0x", sizeof("\nDev 0x"));
					intToHexStr(x, msg_ptr);
					VIRT_UART_Transmit(&huart0, message_address, 2);
					if(ret != DS18B20_OK)
					{
						VIRT_UART_Transmit(&huart0, " Error = 0x", sizeof(" Error = 0x"));
						msg_ptr = message_address;
						intToHexStr(ret, msg_ptr);
						VIRT_UART_Transmit(&huart0, message_address, 2);
					}
					else
					{
						VIRT_UART_Transmit(&huart0, " Success", sizeof(" Success"));
					}
				}

				break;

			case '2':
				VIRT_UART_Transmit(&huart0, "\nStart Temperature Conversion All", sizeof("\nStart Temperature Conversion All"));
				ret = DS18B20_convertTemperatureAll(&devices[0]);
				if(ret != DS18B20_OK)
				{
					VIRT_UART_Transmit(&huart0, " Error = 0x", sizeof(" Error = 0x"));

					msg_ptr = message_address;
					intToHexStr(ret, msg_ptr);
					VIRT_UART_Transmit(&huart0, message_address, 2);
				}
				else
				{
					VIRT_UART_Transmit(&huart0, "\nSuccess", sizeof("\nSuccess"));
				}
				break;

			case '3':
				VIRT_UART_Transmit(&huart0, "\nRead Temperatures", sizeof("\nRead Temperatures"));
				for(int x=0; x<dev_cnt; x++)
				{
					DS18B20_TEMPERATURE temperature;
					ret = DS18B20_getTemperature(&devices[x], &temperature);
					VIRT_UART_Transmit(&huart0, "\nDev 0x", sizeof("\nDev 0x"));

					intToHexStr(x, msg_ptr);
					VIRT_UART_Transmit(&huart0, message_address, 2);

					VIRT_UART_Transmit(&huart0, " Temperature = 0x", sizeof(" Temperature = 0x"));
					msg_ptr = message_address;
					intToHexStr(temperature.integer, msg_ptr);
					VIRT_UART_Transmit(&huart0, message_address, 2);
				}
				break;

			default:
				VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
				break;
			}
			VIRT_UART_Transmit(&huart0, "\n", sizeof("\n"));

		}
	}
}

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

