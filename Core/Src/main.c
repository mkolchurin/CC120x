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
#include <CC120x_register_settings.h>
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

/*
 void setRegisters(void) {
 uint8_t array_size = (sizeof(preferredSettings)
 / sizeof(preferredSettings[0]));
 uint8_t registers[array_size];

 //write settings
 for (int i = 0; i < array_size; i++) {
 CC120x_WriteReg((preferredSettings[i].addr),
 (preferredSettings[i].data));
 }
 HAL_Delay(10);
 //read settings
 for (int i = 0; i < array_size; i++) {
 registers[i] = CC120x_ReadReg((preferredSettings[i].addr));
 }
 printf(registers);
 }
 */
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

	while (1)
	{
/*		cc120x_Init(hspi3, GPIOA, GPIO_PIN_4);

		cc120x_WriteStrobe(SRES);

		HAL_Delay(100);

		uint8_t regs[] =
		{ 0x02, 0x13 };*/
		cc120x_DataTypedef d1, d2, d3, d4;/*
		d1 = cc120x_WriteBurstReg(0x0001, regs, 2);
		d2 = cc120x_WriteSingleReg(0x1234, 0x05);
		d3 = cc120x_ReadSingleReg(0x0001);

		d4 = cc120x_ReadSingleReg(0x0002);
//		d5 = cc120x_ReadSingleReg(0x1234);

		HAL_Delay(100);

		printf(d3.CC120x_Status);
		printf(d3.CC120x_Received);
		printf(d4.CC120x_Status);
		printf(d4.CC120x_Received);*/


		cc120x_Init(hspi3, GPIOB, GPIO_PIN_8);
		cc120x_WriteStrobe(SRES);
		 HAL_Delay(100);
//		 cc120x_WriteStrobe(SIDLE);
//		 cc120x_WriteStrobe(SCAL);
		 cc120x_WriteSettings(preferredSettingsRX);

		 cc120x_WriteStrobe(SRX);


		 cc120x_Init(hspi3, GPIOA, GPIO_PIN_4);
		 cc120x_WriteStrobe(SRES);
//		 cc120x_WriteStrobe(SIDLE);
//		 cc120x_WriteStrobe(SCAL);

		 HAL_Delay(100);


		 cc120x_WriteSettings(preferredSettingsTX);
		 registerSetting_t *rs = cc120x_ReadSettings();

		 HAL_Delay(100);


		 uint8_t txBuffer[] = { 'H', 'E', 'L', 'L', 'O' };
		 cc120x_TransmitData(txBuffer);

		 HAL_Delay(100);


		 cc120x_Init(hspi3, GPIOB, GPIO_PIN_8);
		 cc120x_WriteStrobe(SIDLE);
		 HAL_Delay(100);
		 cc120x_DataTypedef rx = cc120x_ReceiveData();
		 d1 = cc120x_ReadSingleReg(FIFO_NUM_RXBYTES);

		 d2 = cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, StdFIFO, NULL, 5);
		 HAL_Delay(100);

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
