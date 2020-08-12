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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RXLENGTH 9
char *bufftr;
char Rx_data[RXLENGTH];  //  creating a buffer of 10 bytes
int dauerlauf1 = 0;
int dauerlauf2 = 0;

void sendData(uint8_t adresse, uint32_t daten)
{
	//TMC5130 braucht 40 Bit. Davon sind 8 Bit die Adresse und 32 Bit das Datenpaket

// 8 Bit Bspeispiel Hex 0x01 BIN 0000 0001

	uint8_t _adresse = 0;
	uint8_t _paket1 = 0;
	uint8_t _paket2 = 0;
	uint8_t _paket3 = 0;
	uint8_t _paket4 = 0;
	uint8_t paket1, paket2, paket3, paket4;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // Chip für die Kommunikation vorbereiten

	HAL_SPI_Transmit(&hspi2, (uint8_t*) &adresse, 1, 10); // Adresse wird verschicket und der letzte SPI Status wird empfangen
	HAL_SPI_Receive(&hspi2, (uint8_t*) _adresse, 1, 10);

	paket1 = ((daten >> 24) & 0xff);
	HAL_SPI_Transmit(&hspi2, (uint8_t*) &paket1, 1, 10); // Datenpaket erfolgreich geschreiben mit jeweils 8 Bit SPI.transfer
	HAL_SPI_Receive(&hspi2, (uint8_t*) _paket1, 1, 10);

	paket2 = ((daten >> 16) & 0xff);
	HAL_SPI_Transmit(&hspi2, (uint8_t*) &paket2, 1, 10);
	HAL_SPI_Receive(&hspi2, (uint8_t*) _paket2, 1, 10);

	paket3 = ((daten >> 8) & 0xff);
	HAL_SPI_Transmit(&hspi2, (uint8_t*) &paket3, 1, 10);
	HAL_SPI_Receive(&hspi2, (uint8_t*) _paket3, 1, 10);

	paket4 = (daten & 0xff);
	HAL_SPI_Transmit(&hspi2, (uint8_t*) &paket4, 1, 10);
	HAL_SPI_Receive(&hspi2, (uint8_t*) _paket4, 1, 10);
	// Chip für die Kommunikation wieder freigeben
	//__HAL_SPI_DISABLE(&hspi2);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		int pos;
		//char text[RXLENGTH];
		//strcpy(text, Rx_data);
		pos = strcspn(Rx_data, "w");
		char *cmd;
		char *ptr1;
		char *ptr2;
		cmd = strtok(Rx_data, "w");

		ptr1 = strtok(NULL, "w");
		ptr2 = strtok(NULL, "w");

//Datenverarbeitung
		//Motornummer w Programm w Parameter
		//Programm-> 1 Beschleunigung
		//Programm-> 2 Umdrehungen (Positive wie negativ)
		//Programm-> 3 Dauerlauf
		//Programm-> 4 Stop

		//Beispiel-> 1w2w10w
		//Motor 1 soll 10 Umdrehungen machen

		if (atoi(cmd) == 1)
		{		   // Motor 1
			switch (atoi(ptr1))
			{
			case 1:		   // Beschleunigung
				sendData(0xA4, (uint8_t*) atoi(ptr2));		   //A1=1000
				break;
			case 2:		   //Umdrehungen
				// 200 Ganzschritte mal 256 Microschritte = 51200 Schritte pro Umdrehung
				// Fahre x Umdrehungen für Motor 1
				sendData(0xA1, 0x00000000);	// Motor 1 XACTUAL Rücksetzen der Position damit die Umdrehungen stimmen
				sendData(0xAD, (uint32_t*) (atoi(ptr2) * 51200));
				dauerlauf1 = 0;
				break;
			case 3:	//Dauerlauf
				dauerlauf1 = 1;
				sendData(0xAD, 0x00100000);
				break;
			case 4:	//stop
				sendData(0xA1, 0x00000000);	// Motor 1 XACTUAL
				sendData(0xAD, 0x00000000); // Motor 1 XTARGET
				dauerlauf1 = 0;
				break;
			}
		}
		if (atoi(cmd) == 2)
		{ //Motor 2
			switch (atoi(ptr1))
			{
			case 1:		   // Beschleunigung
				sendData(0xC4, (uint8_t*) atoi(ptr2));		   //A1=1000
				break;
			case 2:		   //Umdrehungen
				// 200 Ganzschritte mal 256 Microschritte = 51200 Schritte pro Umdrehung
				// Fahre x Umdrehungen für Motor 2
				sendData(0xC1, 0x00000000);	// Motor 2 XACTUAL Rücksetzen der Position damit die Umdrehungen stimmen
				sendData(0xCD, (uint32_t*) (atoi(ptr2) * 51200));
				dauerlauf2 = 0;
				break;
			case 3:	//Dauerlauf
				dauerlauf2 = 1;
				sendData(0xCD, 0x00100000);
				break;
			case 4:	//stop
				sendData(0xC1, 0x00000000);	// Motor 1 XACTUAL
				sendData(0xCD, 0x00000000); // Motor 1 XTARGET
				dauerlauf2 = 0;
				break;
			}
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t*) Rx_data, RXLENGTH);
	}
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
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) Rx_data, RXLENGTH);

	__HAL_SPI_ENABLE(&hspi2);
	sendData(0x00, 0x00000000); //GCONF Read

	sendData(0x80, 0x00000000); //GCONF

	sendData(0xA0, 0x00000000); // Motor 1 RAMPMODE=0

	sendData(0xC0, 0x00000000); // Motor 2 RAMPMODE=0

	// Motor 1
	sendData(0xA3, 0x0000010F); // VSTART = 15

	sendData(0xA4, 0x000003E8); //A1=1000

	sendData(0xA5, 0x000186A0); //V1=100000

	sendData(0xA6, 0x00000200); //AMAX=50000

	sendData(0xA7, 0x00DFFFFF); // //VMAX=100000 Geschwindigkeit auf MAX

	sendData(0xA8, 0x0000F550); //DMAX

	sendData(0xAA, 0x0000F578); //D1=1400

	sendData(0xAB, 0x00000020); //VSTOP=32

	sendData(0xAC, 0x0000000A); //TPOWERDOWN=10

	// Motor 2
	sendData(0xC3, 0x0000000F); // VSTART = 15

	sendData(0xC4, 0x00000108); //A1=1000

	sendData(0xC5, 0x000056A0); //V1=100000

	sendData(0xC6, 0x0000C350); //AMAX=50000

	sendData(0xC7, 0x000086A0); //VMAX=100000

	sendData(0xC8, 0x000002BC); //DMAX

	sendData(0xCA, 0x00000578); //D1=1400

	sendData(0xCB, 0x00000020); //VSTOP=32

	sendData(0xCC, 0x0000000A); //TPOWERDOWN=10

	//IHOLD Config                                            delay  irun   ihold
	// unsigned long iholdconfig = 0b 0000 0000 0000 0000 00   0011  11110  00000;
	sendData(0xB0, 0b00000000000000000000111111000011);  	// Motor 1

	sendData(0xD0, 0b00000000000000000000111111000011);  	// Motor 2

	//CHOPPER Config
	//sendData(0xEC,0x000101D5);      //CHOPCONF: TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
	sendData(0xEC, 0b00000000000000001000000000000010);  	// Motor 1

	sendData(0xFC, 0b00000000000000001000000000000010);  	// Motor 2

	// PWM Config                                freewheel    autoscale   pwm_freq    pwm_grad   pwm_amplitude
	//unsigned long pwmconfig = 0b 0000 0000  00     01    0      1          01       11111111   11111111;
	sendData(0x90, 0b00000000001001011111111111111111);  	// PWMCONF Motor 1

	sendData(0x98, 0b00000000001001011111111111111111);  	// PWMCONF Motor 2

	sendData(0xA1, 0x00000000);  	// Motor 1 XACTUAL

	sendData(0xAD, 0x00000000); // Motor 1 XTARGET

	sendData(0xC1, 0x00000000); // Motor 2 XACTUAL

	sendData(0xCD, 0x00000000); // Motor 2 XTARGET

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		bufftr = "Hello!\n\r";
		HAL_UART_Transmit(&huart2, (uint8_t*) bufftr, 8, 1);
		HAL_Delay(1000);
		if (dauerlauf1 == 1)
		{
			sendData(0xA1, 0x00000000);	// Motor 1 XACTUAL
		}
		if (dauerlauf2 == 1)
		{
			sendData(0xC1, 0x00000000);	// Motor 1 XACTUAL
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

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
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

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
