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

//void dataAccessTest(void)
//{
//	cc120x_Init(hspi3, GPIOA, GPIO_PIN_4);
//	cc120x_WriteStrobe(SRES);
//	HAL_Delay(100);
//
//	uint16_t address = FS_DIG0;
//	uint8_t txData[] =
//	{ 0xA0 };
//	uint8_t length = 1;
//
//	cs_low();
//	cc120x_DataTypedef dataTX = cc120x_RegAccess(CC120x_Write,
//			CC120x_SingleAccess, address, txData, length);
//
//	cs_high();
//
//	cs_low();
//	cc120x_DataTypedef dataRX = cc120x_RegAccess(CC120x_Read,
//			CC120x_SingleAccess, address, 0, length);
//	cs_high();
//
//	HAL_Delay(100);
//}
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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	cc120x_Init(hspi3, GPIOA, GPIO_PIN_4);
	cc120x_WriteStrobe(SRES);
	HAL_Delay(100);

	cc120x_WriteSettings();
	while (1)
	{
////////RX

		uint8_t addr = 0b11111111;
		uint8_t rx[10] =
		{ 0 };
		rx[5] = 0x1F;
//		while (rx[5] == 0x1F)
//			cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, SNOP, 0, &rx[5], 1);
		while (rx[0] == 0x00)
		{
			cc120x_WriteStrobe(SRX);
			HAL_Delay(300);

			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, NUM_RXBYTES, 0,
					&rx[0], 1);
			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, FIFO_NUM_RXBYTES,
					0, &rx[1], 1);
			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, RXFIFO_PRE_BUF,
					0, &rx[2], 1);
			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, SERIAL_STATUS, 0,
					&rx[3], 1);
			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, FIFO_CFG, 0,
					&rx[4], 1);
			cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, SNOP, 0, &rx[5],
					1);
			HAL_Delay(100);
			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, addr, 0, &rx, 10);
//			cc120x_WriteStrobe(SRX);
//			HAL_Delay(100);
		}
		cc120x_WriteStrobe(SNOP);
//		cc120x_WriteStrobe(SIDLE);

		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, addr, 0, &rx, 10);
		cc120x_WriteStrobe(SNOP);
		cc120x_WriteStrobe(SFRX);

		if (rx[3] != 0x00)
		{
			HAL_Delay(100);
		}

///////////TX
//
//
//
//		uint8_t size = 64;
//		HAL_Delay(100);
//		uint8_t addr = 0b01111111;
//		uint8_t tx[size];
//		for (uint8_t j = 0; j < size; j++)
//		{
//			for (uint8_t i = 0; i < size; i++)
//			{
//				tx[i] = i;
//			}
//			tx[0] = 0xAA;
//			tx[1] = 0xAA;
//			tx[2] = 0xAA;
//			tx[3] = 0xD9;
//			tx[4] = 0xCC;
//
//			cc120x_RegAccess(CC120x_Write, CC120x_burstAccess, addr, tx, 0,
//					size);
//
////			uint8_t rx[10] =
////			{ 0 };
////			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, FIFO_NUM_TXBYTES,
////					0, &rx[0], 1);
//
//			cc120x_WriteStrobe(STX);
//			HAL_Delay(200);
//			cc120x_WriteStrobe(SFTX);
//		}
//
//		/*		while (rx[0] != 0x01)
//		 {
//		 cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, FIFO_NUM_TXBYTES,
//		 0, &rx[0], 1);
//		 HAL_Delay(100);
//		 }*/
//
//		HAL_Delay(100);
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
