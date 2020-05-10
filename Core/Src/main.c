/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CC120x.h"
#include "RFTools.h"
#include "cmp.h"
#include <stdio.h>

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t cc120x_redyToTranfmitt = 1;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

//  HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	cc120x_Init(hspi3, CS_GPIO_Port, CS_Pin);

	cc120x_WriteSettings(preferredSettings, settingsSize());
	cc120x_WriteStrobe(SNOP);

	HAL_Delay(100);
	cc120x_beginReceive();
	/*
	 uint8_t bufsize = 128;
	 uint8_t buffer[bufsize];
	 message_t message;
	 for (int i = 0; i < bufsize; i++)
	 buffer[i] = 0;
	 rftoolsPacket_t packet;
	 uint8_t dataIsOK = 0; */
//	uint8_t radioData[100] = { 0 };
	chip_status_t status = cc120x_WriteStrobe(SRX);
//	cc120x_WriteStrobe(SRX);
//	cc120x_WriteStrobe(SRX);
//	cc120x_WriteStrobe(SRX);
	while (1) {
		TISerialRadio_init(G2_GPIO_Port, G2_Pin, G0_GPIO_Port, G0_Pin, G3_GPIO_Port, G3_Pin);
//		uint8_t rssi = 0;//cc120x_RSSI();

//		status = cc120x_WriteStrobe(SNOP);
//
//		sprintf(buf, "%d - %d\n", rssi, status.chip_status);
//		if (HAL_UART_Transmit_IT(&huart1, buf, sizeof(buf)) == HAL_OK)
//		HAL_UART_Transmit_IT(&huart1, buf, sizeof(buf));

		/* rftools
		 //uint8_t offset = 0;

		 while (dataIsOK == 0) {
		 if (HAL_UART_Receive_IT(&huart1, buffer, PACKET_LENGTH)
		 == HAL_OK) {
		 dataIsOK = getPacket(buffer, &packet);
		 //				break;
		 //			else
		 //				HAL_UART_Abort(&huart1);

		 if (dataIsOK > 0)
		 {HAL_UART_Transmit_IT(&huart1, (uint8_t*) ASK, sizeof(ASK));
		 dataIsOK = 0;
		 }
		 //				break;
		 }
		 }

		 if (dataIsOK > 0) {
		 dataIsOK = 0;
		 HAL_UART_Transmit(&huart1, (uint8_t*) ASK, sizeof(ASK), 0x10);
		 message_t *ptrMessage = &message;
		 uint8_t messageSize = deserialize(packet.Data,
		 (message_t*) ptrMessage);

		 if (message.command == RFTOOLS_COMMAND_REGISTER) {
		 uint16_t naddress = ((uint16_t) message.address[0] << 8)
		 | message.address[1];
		 cc120x_WriteSingleRegister((uint16_t) naddress,
		 message.data[0]);
		 }
		 if (message.command == RFTOOLS_COMMAND_STROBE) {
		 cc120x_WriteStrobe(message.data[0]);
		 }

		 }

		 //
		 //		uint8_t size = cc120x_NumRxBytes();
		 //	if(size != 0 )
		 //		HAL_Delay(2);
		 * */

//HAL_SPI_Receive(&hspi1, radioData, sizeof(radioData), 0xFF);
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

uint64_t syncword = 0xAAAAAAAAAAAAAAAA;
uint64_t recsyncword = 0;
uint8_t sync_word_index = sizeof(syncword) - 1;
uint8_t bitpos = 0;
uint8_t ReceiveSync(uint8_t pinstate) {
//	if (bitpos != 63) {
//		recsyncword |= (pinstate << (63 - bitpos++));
//	} else {
		if (bitpos > 63 && recsyncword == syncword) {
			bitpos = 0;
			return 1;
		}
		recsyncword = (recsyncword << 1) + pinstate;
		bitpos++;
//	}
	return 0;
}

uint16_t valIndex = 0;
uint16_t clkpin = G3_Pin;
GPIO_TypeDef *rxPort = G2_GPIO_Port;
uint16_t rxPin = G2_Pin;
uint8_t syncreceive = 0;
uint8_t counter = 0;

uint8_t transmitt = 0;
void SerialReceive(uint16_t GPIO_Pin, uint8_t *buf, uint8_t length) {
	GPIO_PinState pinstate = HAL_GPIO_ReadPin(rxPort, rxPin);
	if (syncreceive == 0)
//		syncreceive = ReceiveSync(pinstate);
		syncreceive =TISerialRadio_ReceiveSync(syncword);
	else {
		buf[valIndex] += pinstate << (7 - counter++);
		if (counter == 8) {
			counter = 0;
			valIndex++;

			if (valIndex == length) {
				syncreceive = 0;
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				valIndex = 0;
				transmitt = 1;
				cc120x_WriteStrobe(STX);
				cc120x_WriteStrobe(STX);
//				for (int i = 0; i < length; i++)
//					buf[i] = 0;
			}
		}
	}
}

uint8_t bufpos = 0;
uint8_t tcount = 0;
uint8_t state = 0;
uint8_t serialTransmitt(uint8_t *buf, uint8_t size) {
	state = ((buf[bufpos] & (0x80 >> tcount)) >> (7 - tcount));
	tcount++;
	HAL_GPIO_WritePin(G0_GPIO_Port, G0_Pin, state);
	if (tcount == 8) {
		bufpos++;
		tcount = 0;
	}
	if (bufpos == size) {
		bufpos = 0;
		return 1;
	}
	return 0;
}
uint8_t preamble[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
uint8_t buf[255] = { 0 };
uint8_t preambletransmitted = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == clkpin) {
		if (transmitt != 1)
			SerialReceive(GPIO_Pin, buf, 255);
		else {
			if (preambletransmitted == 0) {
				preambletransmitted = serialTransmitt((uint8_t*) preamble,
						sizeof(preamble));
			} else {
				if (1 == serialTransmitt(buf, sizeof(buf)))
					preambletransmitted = 0;
			}
		}
	}

//	if (GPIO_Pin == L1_Pin) {
//		uint8_t pinstate = HAL_GPIO_ReadPin(L2_GPIO_Port, L2_Pin);
//		val = val | (pinstate << tick);
//		tick++;
//		if (tick > 8) {
////			HAL_Delay(1);
//			val = 0;
//			tick = 0;
//		}
//	}
	/*if (GPIO_Pin == G3_Pin) {
	 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	 //cc120x_redyToTranfmitt = 1;
	 uint8_t size = cc120x_NumRxBytes();
	 if (size <= 0)
	 return;
	 //size = 54;
	 uint8_t pData[size];
	 for (int i = 0; i < size; i++) {
	 pData[i] = 0;
	 }

	 chip_status_t receive_status = cc120x_ReceiveData((uint8_t*) &pData,
	 size);
	 uint8_t rssi = cc120x_RSSI();
	 //		cc120x_WriteStrobe(STX);
	 cc120x_WriteStrobe(SIDLE);
	 //		cc120x_WriteStrobe(SFRX);
	 //HAL_Delay(100);

	 //				cc120x_WriteStrobe(SFTX);
	 //		receive_status = cc120x_WriteStrobe(STX);
	 //		receive_status = cc120x_WriteStrobe(SNOP);
	 //
	 //		cc120x_TransmittData(pData, size);

	 //		uint8_t nnl[] = { 134, 170, 105, 150, 166, 169, 170, 170, 85, 90, 13,
	 //				84, 211, 45, 77, 83, 85, 84, 170, 180, 0, 0, 0, 0, 0, 0, 7, };
	 //		cc120x_TransmittData(pData, size);
	 //		receive_status = cc120x_WriteStrobe(SNOP);
	 cc120x_WriteStrobe(SRX);
	 //		cc120x_WriteStrobe(SFRX);
	 //		cc120x_WriteStrobe(SIDLE);
	 //		receive_status = cc120x_beginReceive();

	 //main();
	 }*/
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
