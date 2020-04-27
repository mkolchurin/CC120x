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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	cc120x_Init(hspi3, GPIOA, GPIO_PIN_4);

	cc120x_WriteSettings(preferredSettings, settingsSize());
	cc120x_WriteStrobe(SNOP);

	HAL_Delay(100);
	cc120x_beginReceive();

	uint8_t bufsize = 254;
	uint8_t *buffer = malloc(bufsize);
	message_t message;
	message_t response;

	while (1) {
		for (int i = 0; i < bufsize; i++)
			buffer[i] = 0;
		if (HAL_UART_Receive(&huart1, buffer, bufsize, 0xFF) != HAL_OK) {
			if (buffer[9] == 255)
				buffer[9] = 0;

			message.command = 255;
			deserialize(buffer, &message);
			if (buffer[0] != 0) {
				HAL_Delay(1);
			}
			if (message.command == RFTOOLS_COMMAND_REGISTER) {
				uint16_t naddress = ((uint16_t) message.address[0] << 8)
						| message.address[1];
				cc120x_WriteSingleRegister((uint16_t) naddress,
						message.data[0]);

				response.command = RFTOOLS_COMMAND_RESPONSE;
				response.data[0] = (uint8_t) true;
				uint8_t *buf = malloc(128);
				serialize(response, (uint8_t**) &buf);
				uint8_t length = sizeof(message_t);
				HAL_UART_Transmit(&huart1, buf, length, 0xff);
				HAL_UART_Transmit(&huart1, (uint8_t*) "\n\n\n", 3, 0xff);
				HAL_Delay(1);
			}

			response.command = RFTOOLS_COMMAND_RESPONSE;
			response.data[0] = (uint8_t) false;
			serialize(response, (uint8_t**) &buffer);
			uint8_t length = sizeof(message_t);
			HAL_UART_Transmit(&huart1, buffer, length, 0xff);
			HAL_UART_Transmit(&huart1, (uint8_t*) "\n\n\n", 3, 0xff);


		}

		uint8_t rssi = cc120x_RSSI();
		HAL_Delay(2);
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
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == G3_EXTI_Pin) {
		cc120x_redyToTranfmitt = 1;
		uint8_t size = cc120x_NumRxBytes();
		//size = 254;
		uint8_t pData[size];
		for (int i = 0; i < size; i++)
			pData[i] = 0;

		chip_status_t receive_status = cc120x_ReceiveData(&pData, size);

		cc120x_WriteStrobe(SIDLE);
		cc120x_WriteStrobe(STX);
		cc120x_WriteStrobe(SFTX);
		receive_status = cc120x_WriteStrobe(STX);
		receive_status = cc120x_WriteStrobe(SNOP);

		cc120x_TransmittData(pData, size);
		receive_status = cc120x_WriteStrobe(SNOP);
		//		cc120x_WriteStrobe(SFRX);
		//		cc120x_WriteStrobe(SIDLE);
		receive_status = cc120x_beginReceive();
		message_t message;
		message.command = 2;
		sprintf(message.data, "hello, world!");
		message.data_length = sizeof(message.data);
		message.address[0] = 0xAB;
		message.address[1] = 0xAC;

		uint8_t *buffer = malloc(128);
		uint8_t length = serialize(message, &buffer);

		message_t message1;
		deserialize(buffer, &message1);

		HAL_UART_Transmit(&huart1, buffer, length, 0xff);
		HAL_UART_Transmit(&huart1, (uint8_t*) "\n\n\n", 3, 0xff);
		//uart_transmit(size, 1, pData);
	}
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
