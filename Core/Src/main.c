/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CC120x.h"
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

void createPacket(uint8_t txBuffer[]);
#define ISR_ACTION_REQUIRED 1
#define ISR_IDLE            0
#define RX_FIFO_ERROR       0x11
uint8_t packetSemaphore;
uint32_t packetCounter = 0;

void runRX(void)
{

	uint8_t rxBuffer[128] =
	{ 0 };
	uint8_t rxBytes;
	uint8_t rxBytes1;
	uint8_t marcState;

//    // Connect ISR function to GPIO2
//    ioPinIntRegister(IO_PIN_PORT_1, GPIO2, &radioRxISR);
//
//    // Interrupt on falling edge
//    ioPinIntTypeSet(IO_PIN_PORT_1, GPIO2, IO_PIN_FALLING_EDGE);
//
//    // Clear ISR flag
//    ioPinIntClear(IO_PIN_PORT_1, GPIO2);
//
//    // Enable interrupt
//    ioPinIntEnable(IO_PIN_PORT_1, GPIO2);

// Set radio in RX
	cc120x_WriteStrobe(SFRX);
	HAL_Delay(10);
	cc120x_WriteStrobe(SRX);
	cc120x_WriteStrobe(SRX);

	// Infinite loop
	while (1)
	{
		cc120x_WriteStrobe(SNOP);
		GPIO_PinState s0 = HAL_GPIO_ReadPin(G0_GPIO_Port, G0_Pin);
//		GPIO_PinState s2 = HAL_GPIO_ReadPin(G2_GPIO_Port, G2_Pin);
		GPIO_PinState s3 = HAL_GPIO_ReadPin(G3_GPIO_Port, G3_Pin);
		if (s0 == GPIO_PIN_SET || s3 == GPIO_PIN_SET /*|| s3 == GPIO_PIN_SET*/)
		{
			packetSemaphore = 1;
			while (s0 == GPIO_PIN_SET /*|| s2 == GPIO_PIN_SET*/
			|| s3 == GPIO_PIN_SET)
			{
				s0 = HAL_GPIO_ReadPin(G0_GPIO_Port, G0_Pin);
				/*s2 = HAL_GPIO_ReadPin(G2_GPIO_Port, G2_Pin);*/
				s3 = HAL_GPIO_ReadPin(G3_GPIO_Port, G3_Pin);
			}
//			if (cc120x_WriteStrobe(SNOP) == 0x6F)
//			{
//				cc120x_WriteStrobe(SFRX);
//			}
		}

		//cc120xSpiReadReg(NUM_RXBYTES, &rxBytes, 1);

		//if(rxBytes != 0x00)
		//	packetSemaphore = 1;

		// Mask out MARCSTATE bits and check if we have a RX FIFO error

		// Wait for packet received interrupt
		if (packetSemaphore == ISR_ACTION_REQUIRED)
		{

//			while (s0 == GPIO_PIN_SET || s2 == GPIO_PIN_SET
//					|| s3 == GPIO_PIN_SET)
//			{
//				s0 = HAL_GPIO_ReadPin(G0_GPIO_Port, G0_Pin);
//				s2 = HAL_GPIO_ReadPin(G2_GPIO_Port, G2_Pin);
//				s3 = HAL_GPIO_ReadPin(G3_GPIO_Port, G3_Pin);
//			}
			// Read number of bytes in RX FIFO
			cc120xSpiReadReg(NUM_RXBYTES, &rxBytes, 1);
			cc120xSpiReadReg(RXFIRST, &rxBytes1, 1);

			// Check that we have bytes in FIFO
			if (rxBytes != 0)
			{

				// Read MARCSTATE to check for RX FIFO error
				cc120xSpiReadReg(MARCSTATE, &marcState, 1);

				// Mask out MARCSTATE bits and check if we have a RX FIFO error
				if ((marcState & 0x1F) == RX_FIFO_ERROR)
				{

					// Flush RX FIFO
					cc120x_WriteStrobe(SFRX);
				}
				else
				{

					// Read n bytes from RX FIFO
					cc120xSpiReadReg((uint16_t) 0x3F, &rxBuffer, rxBytes);

					uint8_t rssi0 = 0, rssi1 = 0;
					cc120xSpiReadReg((uint16_t) RSSI0, &rssi0, 1);
					cc120xSpiReadReg((uint16_t) RSSI1, &rssi1, 1);
					float rssi = 0;
					if ((rssi0 & 0b00000001) == 1)
					{
						rssi = (rssi1 << 4) + (rssi0 & 0b01111000);
						rssi = (rssi * 0.0625) - 128;
					}
					uint8_t *str = 0;
					sprintf(str, "%f", rssi);
					HAL_UART_Transmit(&huart1, str, sizeof(str), 0xFF);
//					//for(uint8_t i = 0; i < 127; i++){
//						cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, (uint16_t)(0x00 + i), 0, &rxBuffer[i], 1);
//					}

					//cc120xSpiReadReg((uint16_t)0x3F, &rxBuffer, rxBytes);
					cc120x_WriteStrobe(SFRX);
//					cc120xSpiReadReg(RSSI0, &marcState, 1);
//					cc120xSpiReadReg(RSSI1, &rxBytes, 1);
					// Check CRC ok (CRC_OK: bit7 in second status byte)
					// This assumes status bytes are appended in RX_FIFO
					// (PKT_CFG1.APPEND_STATUS = 1)
					// If CRC is disabled the CRC_OK field will read 1
					if (rxBuffer[rxBytes - 1] & 0x80)
					{

						// Update packet counter
						packetCounter++;

					}
					for (int i = 0; i < rxBytes; i++)
						rxBuffer[i] = 0;
				}

			}
			else
				cc120x_WriteStrobe(SFRX);

			// Reset packet semaphore
			packetSemaphore = ISR_IDLE;

			// Set radio back in RX
			cc120x_WriteStrobe(SFRX);
			cc120x_WriteStrobe(SRX);
		}
	}
}

#define ISR_ACTION_REQUIRED 1
#define ISR_IDLE            0

#define PKTLEN              5 // 1 < PKTLEN < 126
void runTX(void)
{

// Initialize packet buffer of size PKTLEN + 1
	uint8_t txBuffer[PKTLEN + 1] =
	{ 0 };
	cc120x_WriteStrobe(SFTX);
	while (1)
	{

		if (cc120x_WriteStrobe(SNOP) == 0x7F)
		{
			cc120x_WriteStrobe(SFTX);
			cc120x_WriteStrobe(STX);
		}
		// Update packet counter
		packetCounter++;

		// Create a random packet with PKTLEN + 2 byte packet
		// counter + n x random bytes
		createPacket(txBuffer);

		// Write packet to TX FIFO
		cc120xSpiWriteReg(0x3F, txBuffer, sizeof(txBuffer));

		// Strobe TX to send packet
		cc120x_WriteStrobe(STX);
		cc120x_WriteStrobe(SNOP);

		// Wait for interrupt that packet has been sent.
		// (Assumes the GPIO connected to the radioRxTxISR function is
		// set to GPIOx_CFG = 0x06)

		while (packetSemaphore != ISR_ACTION_REQUIRED)
		{
			if (cc120x_WriteStrobe(SNOP) == 0x7F)
			{
				cc120x_WriteStrobe(SFTX);
				cc120x_WriteStrobe(STX);
			}
			GPIO_PinState s1 = HAL_GPIO_ReadPin(G3_GPIO_Port, G3_Pin);

			if (s1 == GPIO_PIN_SET)
			{
				packetSemaphore = ISR_ACTION_REQUIRED;
				while (s1 == GPIO_PIN_SET)
				{
					cc120x_WriteStrobe(SNOP);
					s1 = HAL_GPIO_ReadPin(G3_GPIO_Port, G3_Pin);

				}
			}
		}

		// Clear semaphore flag
		packetSemaphore = ISR_IDLE;

	}
}
void createPacket(uint8_t txBuffer[])
{

	txBuffer[0] = PKTLEN;                           // Length byte
	txBuffer[1] = (uint8_t) (packetCounter >> 8);     // MSB of packetCounter
	txBuffer[2] = (uint8_t) packetCounter;           // LSB of packetCounter

// Fill rest of buffer with random bytes
	for (uint8_t i = 3; i < (PKTLEN + 1); i++)
	{
		txBuffer[i] = (uint8_t) i;
	}
}

static uint8_t cc120x_redyToTranfmitt = 1;
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
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	cc120x_Init(hspi3, GPIOA, GPIO_PIN_4);

	cc120x_WriteSettings();

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	if (!cc120x_ReadSettings())
	{
		HAL_Delay(10);
	}
	cc120x_WriteStrobe(SNOP);
	while (1)
	{
		if (cc120x_redyToTranfmitt == 1)
		{
			uint8_t * pTxData =
			{ 0, 1, 2, 3, 4, 5 };
			if (1 == cc120x_TransmittData(&pTxData, sizeof(pTxData)))
				cc120x_redyToTranfmitt = 0;
		}
		/*
		 uint8_t DevAddress = 0b11100011;
		 uint8_t *pData = 0;

		 while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) DevAddress,
		 (uint8_t*) &pData, (uint16_t) sizeof(pData), (uint32_t) 1000)
		 != HAL_OK)

		 {
		 }

		 while (HAL_I2C_Master_Receive(&hi2c1, (uint16_t) DevAddress,
		 (uint8_t*) &pData, (uint16_t) sizeof(pData), (uint32_t) 1000)
		 != HAL_OK)
		 {

		 }
		 */

		//runRX();
		//runTX();
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == G2_Pin)
	{
		cc120x_redyToTranfmitt = 1;
//		uint8_t *pData;
//		cc120x_ReceiveData(pData);
//		HAL_Delay(100);
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
