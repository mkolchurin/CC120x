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

void RX()
{

	uint8_t rx[10] =
	{ 0 };
	uint8_t addr = 0b11111111;

	uint8_t state = 0xFF;

	//wait rx mode
	while ((state & 0b01110000) != 0b00010000)
	{
		cc120x_WriteStrobe(SFRX);
		cc120x_WriteStrobe(SRX);

		HAL_Delay(100);

		state = cc120x_WriteStrobe(SNOP);
	}
	GPIO_PinState s = HAL_GPIO_ReadPin(G0_GPIO_Port, G0_Pin);
	GPIO_PinState s1 = HAL_GPIO_ReadPin(G2_GPIO_Port, G2_Pin);
	while (s1 != GPIO_PIN_SET || rx[0] != 0 || rx[1] != 0)
	{
		s1 = HAL_GPIO_ReadPin(G2_GPIO_Port, G2_Pin);
		state = cc120x_WriteStrobe(SNOP);

		if ((state & 0b01110000) != 0b00010000)
			HAL_Delay(100);
		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, RXFIRST, 0, &rx[0],
				1);
		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, RXLAST, 0, &rx[1],
				1);

		cc120x_WriteStrobe(SNOP);

	}

	cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, RXFIFO_PRE_BUF, 0,
			&rx[2], 1);
	cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, SERIAL_STATUS, 0, &rx[3],
			1);

	cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, addr, 0, &rx, 10);

	cc120x_WriteStrobe(SNOP);
//		cc120x_WriteStrobe(SIDLE);

	cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, addr, 0, &rx, 10);
	cc120x_WriteStrobe(SNOP);
	cc120x_WriteStrobe(SFRX);

	if (rx[3] != 0x00)
	{
		HAL_Delay(100);
	}
}
void TX(){

		cc120x_WriteStrobe(SFTX);
		cc120x_WriteStrobe(SIDLE);
		uint8_t size = 0x128;

		uint8_t addr = 0b01111111;
		uint8_t tx[size];
		for (uint8_t i = 0; i < size; i++)
		{
			if(i == 0x0) {
				tx[i] = size;
				i++;
				continue;
			}
			tx[i] = i;
		}
		cc120x_RegAccess(CC120x_Write, CC120x_burstAccess,
				 addr, tx, 0, size);


		uint8_t state = 0xFF;

		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, TXFIRST, 0,
						&state, 1);
		cc120x_WriteStrobe(STX);
		HAL_Delay(200);
		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, TXFIRST, 0,
						&state, 1);
		state = cc120x_WriteStrobe(SNOP);
				HAL_Delay(200);

		state = cc120x_WriteStrobe(SNOP);
		HAL_Delay(1000);
		/*

		 for (uint8_t j = 0; j < size; j++)
		 {

		 tx[0] = 0xAA;
		 tx[1] = 0xAA;
		 tx[2] = 0xAA;
		 tx[3] = 0xD9;
		 tx[4] = 0xCC;

		 cc120x_RegAccess(CC120x_Write, CC120x_burstAccess, addr, tx, 0,
		 size);

		 //			uint8_t rx[10] =
		 //			{ 0 };
		 //			cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, FIFO_NUM_TXBYTES,
		 //					0, &rx[0], 1);

		 cc120x_WriteStrobe(STX);
		 HAL_Delay(200);
		 cc120x_WriteStrobe(SFTX);
		 }

		 while (rx[0] != 0x01)
		 {
		 cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, FIFO_NUM_TXBYTES,
		 0, &rx[0], 1);
		 HAL_Delay(100);
		 }

		 HAL_Delay(100);*/
}



#define uint8 uint8_t
#define uint32 uint32_t
#define ISR_ACTION_REQUIRED 1
#define ISR_IDLE            0

#define PKTLEN      32 //
#define LABLE 'Z'
#define SOURCE      0x01
#define INDEX '*'  // OK ? request answer
#define DEV_ADDR     0xB0
uint8 packetSemaphore = ISR_IDLE;


uint32 packetCounter  = 0;
uint8 interruptCount = 0;
static void runTX()
{

   static uint8_t marcState;
   uint8_t temp = 2;

// Initialize packet buffer of size PKTLEN + 1
   uint8_t txBuffer[PKTLEN]={
0
};
   temp = interruptCount;

    //Calibrate frequency synthesizer
   cc120xSpiCmdStrobe(SCAL);
   do{
//   after Calibrate frequency synthesizer goto Idle
      cc120xSpiReadReg(MARCSTATE,&marcState,1);  //  read the Marcstate(0x2F73) reg of value

}while(marcState != 0x41); // 0100 0001  bit6:5 10 Idle bit4:0 0001 Idle

   cc120xSpiCmdStrobe(SFSTXON);

   //Loop
   while(1)
   {

  // Create a packet with + 2 bytes  + 26 Abc bytes
  createPacket(txBuffer);
  cc120xSpiReadReg(NUM_TXBYTES,&temp,1);
   // Write packet to TX FIFO
  cc120xSpiWriteTxFifo(txBuffer,sizeof(txBuffer));
  cc120xSpiReadReg(NUM_TXBYTES,&temp,1);

  // Strobe TX to send packet
 //temp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
  cc120xSpiCmdStrobe(STX);
//  cc120xSpiReadReg(CC1200_MARCSTATE,&marcState,1);

  // Wait for interrupt that packet has been sent.
      // (Assumes the GPIO connected to the radioRxTxISR function is set
      // to GPIOx_CFG = 0x06)
  while( packetSemaphore != ISR_ACTION_REQUIRED)
  ;
  cc120xSpiCmdStrobe(SIDLE);//  Send over  and goto SIDLE State
  cc120xSpiCmdStrobe(SFTX); //Flush the Tx FIFO  if not will full
  // Clear semaphore flag
      packetSemaphore = ISR_IDLE;
  temp = interruptCount;
  delay(200);
//  printf("%d\n", interruptCount);


} //end of while(true)

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
//RX();
//TX();
runTX();

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
